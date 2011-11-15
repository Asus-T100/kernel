/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include "atomisp_tables.h"
#include "atomisp_ioctl.h"
#include "atomisp_cmd.h"
#include "atomisp_fops.h"
#include "hrt/css_receiver_ahb_defs.h"
#include "css/sh_css_debug.h"
#include "css/sh_css_hrt.h"
#include "css/sh_css_binary_info.h"
#include "css/sh_css_accelerate.h"
#include "hrt/bits.h"
#include "linux/intel_mid_pm.h"
#include <linux/kernel.h>

/* We should never need to run the flash for more than 2 frames.
 * At 15fps this means 133ms. We set the timeout a bit longer.
 * Each flash driver is supposed to set its own timeout, but
 * just in case someone else changed the timeout, we set it
 * here to make sure we don't damage the flash hardware. */
#define FLASH_TIMEOUT 800 /* ms */

union host {
	struct {
		void *kernel_ptr;
		void __user *user_ptr;
		int size;
	} scalar;
	struct {
		void *hmm_ptr;
	} ptr;
};

static void
atomisp_acc_fw_free_args(struct atomisp_device *isp, struct sh_css_acc_fw *fw);
static void
atomisp_acc_fw_free(struct atomisp_device *isp, struct sh_css_acc_fw *fw);

/*
 * get sensor:dis71430/ov2720 related info from v4l2_subdev->priv data field.
 * subdev->priv is set in mrst.c
 */
struct camera_mipi_info *atomisp_to_sensor_mipi_info(struct v4l2_subdev *sd)
{
	return (struct camera_mipi_info *)v4l2_get_subdev_hostdata(sd);
}

/*
 * get struct atomisp_video_pipe from v4l2 video_device
 */
struct atomisp_video_pipe *atomisp_to_video_pipe(struct video_device *dev)
{
	return (struct atomisp_video_pipe *)
	    container_of(dev, struct atomisp_video_pipe, vdev);
}

/*
 * interrupt enable/disable functions
 */
static void enable_isp_irq(enum hrt_isp_css_irq irq, bool enable)
{
	if (enable) {
		sh_css_hrt_irq_enable(irq, true, false);
		switch (irq) { /*We only have sp interrupt right now*/
		case hrt_isp_css_irq_sp:
			sh_css_hrt_irq_enable_sp(true);
			break;
		default:
			break;
		}

	} else {
		sh_css_hrt_irq_disable(irq);
		switch (irq) {
		case hrt_isp_css_irq_sp:
			sh_css_hrt_irq_enable_sp(false);
			break;
		default:
			break;
		}
	}

}

/*
 * interrupt clean function
 */
static void clear_isp_irq(enum hrt_isp_css_irq irq)
{
	switch (irq) {
	case hrt_isp_css_irq_sp:
		sh_css_hrt_irq_clear_sp();
	default:
		break;
	}
}

void atomisp_msi_irq_init(struct atomisp_device *isp, struct pci_dev *dev)
{
	u32 msg_ret;
	u32 msi_address;
	u16 msi_data;

	pci_read_config_dword(dev, PCI_MSI_ADDR, &msi_address);
	pci_read_config_word(dev, PCI_MSI_DATA, &msi_data);
	msi_data = msi_data & 0xffff;

	atomisp_msg_write32(isp, IUNIT_PORT, MSI_ADDRESS, msi_address);

	msg_ret = atomisp_msg_read32(isp, IUNIT_PORT, MSI_CAPID);
	msg_ret |= 1 << MSI_ENABLE_BIT;
	atomisp_msg_write32(isp, IUNIT_PORT, MSI_CAPID, msg_ret);

	msg_ret = (1 << INTR_IER) | (1 << INTR_IIR);
	atomisp_msg_write32(isp, IUNIT_PORT, INTR_CTL, msg_ret);

	msg_ret = atomisp_msg_read32(isp, IUNIT_PORT, PCICMDSTS);
	msg_ret |= (1 << MEMORY_SPACE_ENABLE |
		    1 << BUS_MASTER_ENABLE |
		    1 << INTR_DISABLE_BIT);
	atomisp_msg_write32(isp, IUNIT_PORT, PCICMDSTS, msg_ret);

	atomisp_msg_write32(isp, IUNIT_PORT, MSI_DATA, msi_data);
}

void atomisp_msi_irq_uninit(struct atomisp_device *isp, struct pci_dev *dev)
{
	u32 msg_ret;

	msg_ret = atomisp_msg_read32(isp, IUNIT_PORT, MSI_CAPID);
	msg_ret &=  ~(1 << MSI_ENABLE_BIT);
	atomisp_msg_write32(isp, IUNIT_PORT, MSI_CAPID, msg_ret);

	msg_ret = 0x0;
	atomisp_msg_write32(isp, IUNIT_PORT, INTR_CTL, msg_ret);

	msg_ret = atomisp_msg_read32(isp, IUNIT_PORT, PCICMDSTS);
	msg_ret &= ~(1 << INTR_DISABLE_BIT |
		     1 << BUS_MASTER_ENABLE);
	atomisp_msg_write32(isp, IUNIT_PORT, PCICMDSTS, msg_ret);
}

static void print_csi_rx_errors(void)
{
	u32 infos = 0;
	sh_css_rx_get_interrupt_info(&infos);

	v4l2_info(&atomisp_dev, "CSI Receiver errors:\n");
	if (infos & SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		v4l2_err(&atomisp_dev, "  buffer overrun");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_SOT)
		v4l2_err(&atomisp_dev,
				"  start-of-transmission error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		v4l2_err(&atomisp_dev,
				"  start-of-transmission sync error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_CONTROL)
		v4l2_err(&atomisp_dev, "  control error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		v4l2_err(&atomisp_dev, "  2 or more ECC errors");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_CRC)
		v4l2_err(&atomisp_dev, "  CRC mismatch");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		v4l2_err(&atomisp_dev, "  unknown error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		v4l2_err(&atomisp_dev, "  frame sync error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		v4l2_err(&atomisp_dev, "  frame data error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		v4l2_err(&atomisp_dev, "  data timeout");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		v4l2_err(&atomisp_dev,
				"  unknown escape command entry");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		v4l2_err(&atomisp_dev, "  line sync error");
}

/* interrupt handling function*/
irqreturn_t atomisp_isr(int irq, void *dev)
{
	u32 msg_ret;
	struct atomisp_device *isp = (struct atomisp_device *)dev;
	enum sh_css_err err;
	unsigned int irq_infos = 0;
	bool signal_worker = false;
	bool signal_statistics = false;
	bool signal_acceleration = false;
	unsigned long irqflags;

	/*
	 * spin_lock_irqsave() is necessary to avoid racing between multiple
	 * calls of ISP's ISR
	 */
	spin_lock_irqsave(&isp->irq_lock, irqflags);
	/*got triggered interrupt*/
	err = sh_css_translate_interrupt(&irq_infos);
	if (err != sh_css_success) {
		v4l2_warn(&atomisp_dev, "%s:failed to translate irq (err = %d,"
			  " infos = %d)\n", __func__, err, irq_infos);
		spin_unlock_irqrestore(&isp->irq_lock, irqflags);
		return IRQ_NONE;
	}

	isp->irq_infos |= irq_infos;
	if (irq_infos & SH_CSS_IRQ_INFO_FRAME_DONE ||
	    irq_infos & SH_CSS_IRQ_INFO_START_NEXT_STAGE) {
		/* Wake up sleep thread for next binary */
		signal_worker = true;
		if (irq_infos & SH_CSS_IRQ_INFO_STATISTICS_READY) {
			signal_statistics = true;
			isp->isp3a_stat_ready = true;
		}
		if (irq_infos & SH_CSS_IRQ_INFO_FW_ACC_DONE) {
			signal_acceleration = true;
		}
	} else if (irq_infos & SH_CSS_IRQ_INFO_CSS_RECEIVER_ERROR) {
		/* handle mipi receiver error*/
		u32 rx_infos;
		print_csi_rx_errors();
		sh_css_rx_get_interrupt_info(&rx_infos);
		sh_css_rx_clear_interrupt_info(rx_infos);
		if (rx_infos & SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN) {
			signal_worker = true;
			signal_statistics = true;
			signal_acceleration = true;
		}
	}

	if (irq_infos & SH_CSS_IRQ_INFO_INVALID_FIRST_FRAME)
		isp->sw_contex.invalid_frame = true;

	/*
	 * After every iteration of acceleration there will be an interrupt
	 * which needs priority.
	 */
	if (signal_acceleration)
		/* wake up acceleration api */
		complete(&isp->acc_fw_complete);

	if (signal_worker)
		/*make work queue run*/
		complete(&isp->wq_frame_complete);
	if (signal_statistics)
		/*make work queue run*/
		complete(&isp->dis_state_complete);

	/* Clear irq reg at PENWELL B0 */
	msg_ret = atomisp_msg_read32(isp, IUNIT_PORT, INTR_CTL);
	msg_ret |=  (1 << INTR_IIR);
	atomisp_msg_write32(isp, IUNIT_PORT, INTR_CTL, msg_ret);

	spin_unlock_irqrestore(&isp->irq_lock, irqflags);
	return IRQ_HANDLED;
}
/*
 * dequeue a buffer from video buffer list
 * if no buffer queued, wait for queue_buf is called
 * will return 1 when streaming off.
 */
static int atomisp_buf_pre_dequeue(struct atomisp_video_pipe *pipe,
			      struct videobuf_buffer **vb)
{
	unsigned long flags;
	struct atomisp_device *isp = pipe->isp;

	spin_lock_irqsave(&pipe->irq_lock, flags);
	if (list_empty(&pipe->activeq)) {
		spin_unlock_irqrestore(&pipe->irq_lock, flags);
		if (wait_event_interruptible(pipe->capq.wait,
					     (!list_empty(&pipe->activeq) &&
					      !isp->sw_contex.updating_uptr) ||
					     !isp->sw_contex.isp_streaming))
			return -EINVAL;

		spin_lock_irqsave(&pipe->irq_lock, flags);
		if (list_empty(&pipe->activeq)) {
			spin_unlock_irqrestore(&pipe->irq_lock, flags);
			return -EINVAL;
		}
	}

	if (!isp->sw_contex.isp_streaming) {
		spin_unlock_irqrestore(&pipe->irq_lock, flags);
		return 1;
	}

	/*retrieve video buffer from activeq, keep video buffer on capq*/
	*vb = list_entry(pipe->activeq.next, struct videobuf_buffer, queue);
	list_del(&(*vb)->queue);
	(*vb)->state = VIDEOBUF_ACTIVE;
	spin_unlock_irqrestore(&pipe->irq_lock, flags);
	return 0;
}

/* set the bit range [lsb, lsb+num_bits] of in_val to new_val
 * and return the result.
 */
static uint32_t u32_set_bits(uint32_t in_val, unsigned int lsb,
	     unsigned int num_bits, uint32_t new_val)
{
	unsigned int bit_mask;
	/* avoid overflow in calculating the bit masks */
	if (num_bits > 32)
		return in_val;
	if (num_bits == 32)
		return new_val;
	/* calculate number of 1 bits in mask */
	bit_mask = (1<<(num_bits+1)) - 1;
	/* make sure new_val doesn't contain more than num_bits */
	new_val &= bit_mask;
	/* shift bits into the target place */
	bit_mask <<= lsb;
	/* clear old bits */
	in_val &= ~bit_mask;
	/* assign new_val to the output position */
	in_val |= new_val << lsb;
	return in_val;
}

/* The term_en_count is set via 2 registers, one for the 4lane port and one
   for the 1lane port. */
#define TERM_EN_COUNT_1LANE_START_BIT 16
#define PNW_B0_TERM_EN_COUNT_4LANE_START_BIT 20
#define PNW_B0_TERM_EN_COUNT_NUM_BITS        4
#define TERM_EN_COUNT_4LANE_START_BIT 24
#define TERM_EN_COUNT_NUM_BITS        7

static void set_term_en_count(struct atomisp_device *isp, int b0)
{
	uint32_t val;
	unsigned int lane1_start_bit,
		     lane4_start_bit,
		     num_bits;

	lane1_start_bit = TERM_EN_COUNT_1LANE_START_BIT;
	if (b0) {
		lane4_start_bit = PNW_B0_TERM_EN_COUNT_4LANE_START_BIT;
		num_bits = PNW_B0_TERM_EN_COUNT_NUM_BITS;
	} else {
		lane4_start_bit = TERM_EN_COUNT_4LANE_START_BIT;
		num_bits = TERM_EN_COUNT_NUM_BITS;
	}

	val = atomisp_msg_read32(isp, IUNITPHY_PORT, CSI_CONTROL);
	/* 1 lane CSI port */
	val = u32_set_bits(val, lane1_start_bit, num_bits, 0xF);
	/* 4 lane CSI port */
	val = u32_set_bits(val, lane4_start_bit, num_bits, 0xF);
	atomisp_msg_write32(isp, IUNITPHY_PORT, CSI_CONTROL, val);
}

static int atomisp_buffer_dequeue(struct atomisp_device *isp,
			    struct videobuf_buffer **vb_capture,
			    struct videobuf_buffer **vb_preview)
{
	struct videobuf_vmalloc_memory *vmem;
	struct atomisp_video_pipe *vf_pipe = NULL;
	struct atomisp_video_pipe *mo_pipe = NULL;
	int ret;

	if (!(*vb_capture)) {
		mo_pipe = &isp->isp_subdev.video_out_mo;
		ret = atomisp_buf_pre_dequeue(mo_pipe, vb_capture);
		if (ret)
			return -EINVAL;

		vmem = (*vb_capture)->priv;
		if (vmem->vaddr == NULL)
			return -EINVAL;

		/*frame structure is stored in videobuf->priv->vaddr*/
		isp->regular_output_frame = vmem->vaddr;
	}

	if (atomisp_is_viewfinder_support(isp) && !(*vb_preview)) {
		vf_pipe = &isp->isp_subdev.video_out_vf;
		ret = atomisp_buf_pre_dequeue(vf_pipe, vb_preview);
		if (ret)
			return -EINVAL;

		vmem = (*vb_preview)->priv;
		isp->vf_frame = vmem->vaddr;
	}
	return 0;
}

static int atomisp_start_binary(struct atomisp_device *isp)
{
	int ret;

	/*Start the isp binary*/
	switch (isp->sw_contex.run_mode) {
	case CI_MODE_STILL_CAPTURE:
		ret = sh_css_capture_start(isp->raw_output_frame,
					   isp->regular_output_frame,
					   isp->vf_frame);
		if (sh_css_success != ret) {
			v4l2_err(&atomisp_dev,
				 "start capture error!\n");

			return -EINVAL;
		}
		break;
	case CI_MODE_PREVIEW:
		sh_css_preview_start(NULL, isp->regular_output_frame);
		break;
	case CI_MODE_VIDEO:
		if (isp->params.video_dis_en) {
			sh_css_video_set_dis_vector(isp->params.dis_x,
						    isp->params.dis_y);
		}
		sh_css_video_start(NULL, isp->regular_output_frame,
				   isp->vf_frame);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int atomisp_streamon_input(struct atomisp_device *isp)
{
	int is_b0, ret;

	if (isp->sw_contex.file_input) {
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 1);
			return ret;
	}

	if (isp->sw_contex.sensor_streaming == false) {
		is_b0 = (isp->pdev->revision == 6) ? 0 : 1;
		set_term_en_count(isp, is_b0);
		/*
		 * stream on the sensor, power on is called before
		 * work queue start
		 */
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 1);
		if (ret)
			return -EINVAL;

		isp->sw_contex.sensor_streaming = true;
	}
	return 0;
}

static void atomisp_pipe_reset(struct atomisp_device *isp)
{
	v4l2_warn(&atomisp_dev, "ISP timeout. Recovering\n");

	/* clear irq */
	enable_isp_irq(hrt_isp_css_irq_sp, false);
	clear_isp_irq(hrt_isp_css_irq_sp);
	/* stream off sensor */
	if (!isp->sw_contex.file_input) {
		v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				 video, s_stream, 0);
		isp->sw_contex.sensor_streaming = false;
	}
	/* power cycle ISP */
	sh_css_suspend();
	pm_runtime_put_sync(isp->dev);
	pm_runtime_get_sync(isp->dev);
	sh_css_resume();
}

static int atomisp_timeout_handler(struct atomisp_device *isp, int cnt)
{
	u32 rx_infos;
	char debug_context[64];

	v4l2_err(&atomisp_dev, "ISP timeout\n");
	isp->isp_timeout = true;

	snprintf(debug_context, 64, "ISP timeout encountered (%d of 5)", cnt);
	sh_css_dump_debug_info(debug_context);

	/* Try 5 times for autorecovery */
	if (cnt < 5) {
		atomisp_pipe_reset(isp);
		return 0;
	}

	/* Can't recover from isp timeout, error will be reported*/
	if (!sh_css_hrt_sp_is_idle())
		v4l2_err(&atomisp_dev,
			 "error: SP is not idle\n");
	else
		v4l2_err(&atomisp_dev,
			 "error: lost interrupt\n");

	print_csi_rx_errors();
	sh_css_rx_get_interrupt_info(&rx_infos);
	sh_css_rx_clear_interrupt_info(rx_infos);
	return -ETIMEDOUT;
}

static bool atomisp_flash_error(struct atomisp_device *isp)
{
	struct v4l2_control ctrl;

	ctrl.id = V4L2_CID_FLASH_STATUS;
	if (v4l2_subdev_call(isp->flash, core, g_ctrl, &ctrl))
		return true;
	if (ctrl.value != ATOMISP_FLASH_STATUS_OK)
		v4l2_err(&atomisp_dev, "flash failed (%d)\n", ctrl.value);
	return ctrl.value != ATOMISP_FLASH_STATUS_OK;
}

static int atomisp_stop_flash(struct atomisp_device *isp)
{
	struct v4l2_control ctrl;

	ctrl.id = V4L2_CID_FLASH_STROBE;
	ctrl.value = 0;
	if (v4l2_subdev_call(isp->flash, core, s_ctrl, &ctrl)) {
		v4l2_err(&atomisp_dev, "flash strobe off failed\n");
		return -EINVAL;
	}

	/* switch flash mode into flash off */
	ctrl.id = V4L2_CID_FLASH_MODE;
	ctrl.value = ATOMISP_FLASH_MODE_OFF;

	if (v4l2_subdev_call(isp->flash, core, s_ctrl, &ctrl)) {
		v4l2_err(&atomisp_dev, "flash mode switch failed\n");
		return -EINVAL;
	}

	return 0;
}

static int atomisp_start_flash(struct atomisp_device *isp)
{
	struct v4l2_control ctrl;

	/* make sure the timeout is set before setting flash mode */
	ctrl.id = V4L2_CID_FLASH_TIMEOUT;
	ctrl.value = FLASH_TIMEOUT;

	if (v4l2_subdev_call(isp->flash, core, s_ctrl, &ctrl)) {
		v4l2_err(&atomisp_dev, "flash timeout configure failed\n");
		return -EINVAL;
	}

	/* switch flash into flash mode */
	ctrl.id = V4L2_CID_FLASH_MODE;
	ctrl.value = ATOMISP_FLASH_MODE_FLASH;

	if (v4l2_subdev_call(isp->flash, core, s_ctrl, &ctrl)) {
		v4l2_err(&atomisp_dev, "flash mode switch failed\n");
		return -EINVAL;
	}

	/* trigger the flash */
	ctrl.id = V4L2_CID_FLASH_STROBE;
	ctrl.value = 1;
	/* for now we treat it as on/off */
	if (v4l2_subdev_call(isp->flash, core, s_ctrl, &ctrl)) {
		v4l2_err(&atomisp_dev, "flash strobe on failed\n");
		return -EINVAL;
	}
	return 0;
}

#define DEFAULT_ISP_TIMEOUT	1
/*
 * this function calculate isp timeout threshold according to sensor's fps
 *
 * this is the algorithm:
 * 1: if sensor's fps > 2, timeout threshold is 1s
 * 2: if sensor's  1 < fps <= 2. timeout threshold is 2s
 * 3: if sensor's fps <= 1, timeout threshold is 1 + 1/fps
 *
 * if no fps can be acquired from sensor, just use the default ISP timeout
 * value
 */
static unsigned int atomisp_update_timeout_val(struct atomisp_device *isp)
{
	struct v4l2_subdev_frame_interval frame_interval;
	unsigned short timeout_val, fps;

	/* check sensor's actual fps */
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
		video, g_frame_interval, &frame_interval)) {
		/*
		 * senor does not support this interface
		 * so no change to default timeout value
		 */
		timeout_val = 1;
	} else {
		fps = frame_interval.interval.denominator /
		    frame_interval.interval.numerator;
		if (fps > 2)
			timeout_val = 1;
		else if (fps > 1)
			timeout_val = 2;
		else { /* frame_rate must be lower than 1 */
			fps = frame_interval.interval.numerator /
			    frame_interval.interval.denominator;
			timeout_val = 1 + fps;
		}
	}

	return timeout_val;
}

void atomisp_work(struct work_struct *work)
{
	struct atomisp_device *isp = container_of(work, struct atomisp_device,
						  work);
	struct videobuf_buffer *vb_preview = NULL;
	struct videobuf_buffer *vb_capture = NULL;
	int ret, timeout_cnt = 0, fps_time;
	bool timeout_flag,
	     flash_in_progress = false,
	     flash_enabled = false;
	enum atomisp_frame_status fr_status = ATOMISP_FRAME_STATUS_OK;
	u32 irq_infos;

	isp->sw_contex.error = false;
	isp->sw_contex.invalid_frame = false;
	INIT_COMPLETION(isp->wq_frame_complete);

	for (;;) {
		timeout_flag = false;
		/*
		 * we check whether invalid_frame is set, if so, we do not
		 * request a new buffer for ISP, just use current buffer, which
		 * is not returned to the user space.
		 *
		 * whether the frame is invalid is only known after ISP
		 * already runs a time and output a frame. so invalid_frame is
		 * firstly initialzed to false, which is changed in isr by
		 * info from ISP firmware if the frame is invalid, after first
		 * time ISP runs and so on.
		 *
		 * There is no need to hold a lock when modify the value of
		 * invalid_frame, though the other place in isr also modify
		 * this value. As at this point, ISP is not run and no irq for
		 * isp would be generated.
		 */
		if (!isp->sw_contex.invalid_frame) {
			ret = atomisp_buffer_dequeue(isp, &vb_capture,
					     &vb_preview);
			if (ret)
				goto error;
		} else {
			isp->sw_contex.invalid_frame = false;
		}

		/* Hold the locker when ISP is running, when dis and digital
		 * zoom can not be configured
		 */
		mutex_lock(&isp->isp_lock);

		/* restore isp normal status */
		isp->isp_timeout = false;

		ret = atomisp_start_binary(isp);
		if (ret) {
			mutex_unlock(&isp->isp_lock);
			goto error;
		}

		ret = atomisp_streamon_input(isp);
		if (ret) {
			mutex_unlock(&isp->isp_lock);
			v4l2_err(&atomisp_dev,
				 "stream on input error.\n");
			goto error;
		}

		/* if the previous frame was partially exposed, this one is
		 * going to be fully exposed. */
		if (flash_in_progress &&
		    fr_status == ATOMISP_FRAME_STATUS_FLASH_PARTIAL) {
			/* If flash is in progress and the previous frame
			 * was partially exposed, then this frame will be
			 * correctly exposed. */
			fr_status = ATOMISP_FRAME_STATUS_FLASH_EXPOSED;
		} else if (flash_in_progress &&
			   fr_status == ATOMISP_FRAME_STATUS_FLASH_EXPOSED) {
			/* If the previous frame was flash-exposed, we assume
			 * that some of the flash leaked into the current frame
			 * so we tell the app not to use this frame. */
			fr_status = ATOMISP_FRAME_STATUS_FLASH_PARTIAL;
			flash_in_progress = false;
		} else {
			fr_status = ATOMISP_FRAME_STATUS_OK;
		}

		if (isp->params.num_flash_frames) {
			int ret = atomisp_start_flash(isp);

			/* The flash failed frame status is needed to signal to
			 * the appication that flash has been attempted but no
			 * flash frames will be generated. Without this, the
			 * applicaton would end up an in endless loop waiting
			 * for a flash-exposed frame.
			 */
			if (ret)
				fr_status = ATOMISP_FRAME_STATUS_FLASH_FAILED;
			else {
				fr_status = ATOMISP_FRAME_STATUS_FLASH_PARTIAL;
				flash_in_progress = true;
				flash_enabled = true;
			}
			isp->params.num_flash_frames = 0;
		}

		/*Check ISP processing pipeline if any binary left*/
		do {
			long time_left;
			unsigned long irqflags;

			/* wait for different binary to be processed */
			spin_lock_irqsave(&isp->irq_lock, irqflags);
			if (!isp->irq_infos) {
				spin_unlock_irqrestore(&isp->irq_lock,
						       irqflags);
				time_left =
					wait_for_completion_timeout(
						&isp->wq_frame_complete,
						DEFAULT_ISP_TIMEOUT * HZ);
				spin_lock_irqsave(&isp->irq_lock, irqflags);
			} else {
				/*
				 * Don't need to sleep because new interrupt was
				 * triggered during last iteration.
				 * time_left needs to be > 0 as it's not
				 * timing out.
				 */
				time_left = 1;
			}

			irq_infos = isp->irq_infos;
			isp->irq_infos = 0;
			spin_unlock_irqrestore(&isp->irq_lock, irqflags);

			if (time_left == 0) {
				WARN(irq_infos, "ATOMISP: ISR out of sync with "
						"Workqueue.\n");
				/*
				 * if the default timeout happens, we see
				 * whether it is caused by sensor's output fps
				 * is too low
				 */
				fps_time = atomisp_update_timeout_val(isp);
				if (fps_time > DEFAULT_ISP_TIMEOUT) {
					/*
					 * sensor's fps is too low, so it is
					 * probably not error timeout, we
					 * need to wait more time
					 */
					time_left =
					wait_for_completion_timeout(
					&isp->wq_frame_complete,
					(fps_time - DEFAULT_ISP_TIMEOUT) * HZ);

					if (time_left == 0)
						/* this is a real timeout now */
						goto timeout_handle;
				} else {
					/*
					 * sensor's fps is high enough, this
					 * is a real timeout
					 */
timeout_handle:
					ret = atomisp_timeout_handler(isp,
							      timeout_cnt++);
					if (ret) {
						mutex_unlock(&isp->isp_lock);
						goto error;
					}
					timeout_flag = true;
					break;
				}
			}

			INIT_COMPLETION(isp->acc_fw_complete);
			if (irq_infos & SH_CSS_IRQ_INFO_FW_ACC_DONE)
				sh_css_terminate_firmware();

			/*
			 * If there is any firware marked for deletion, do it
			 * now. Firware cannot be unloaded in the middle of a
			 * frame, so it has to be done when a frame done
			 * interrupt is processed.
			 * TODO: Check if this is the correct location.
			 * TODO: Check how to handle multiple firmware cases.
			 */
			if (isp->marked_fw_for_unload != NULL) {
				atomisp_acc_fw_free_args(isp,
						    isp->marked_fw_for_unload);
				sh_css_unload_acceleration(
						isp->marked_fw_for_unload);
				atomisp_acc_fw_free(isp,
						    isp->marked_fw_for_unload);
				isp->marked_fw_for_unload = NULL;
				complete(&isp->acc_unload_fw_complete);
			}

			/* regardless of timeout or not, we disable the flash */
			if (flash_enabled && fr_status ==
					ATOMISP_FRAME_STATUS_FLASH_EXPOSED) {
				atomisp_stop_flash(isp);
				/* always check the result, this clears any
				 * errors that may have occurred.
				 */
				if (atomisp_flash_error(isp))
					fr_status =
					ATOMISP_FRAME_STATUS_FLASH_FAILED;
				flash_enabled = false;
			}

			/* proc interrupt */
			INIT_COMPLETION(isp->wq_frame_complete);
			if (irq_infos & SH_CSS_IRQ_INFO_START_NEXT_STAGE) {
				sh_css_start_next_stage();

				/* Getting 3A statistics if ready */
				if (isp->isp3a_stat_ready) {
					mutex_lock(&isp->isp3a_lock);
					ret = sh_css_get_3a_statistics
						(isp->params.s3a_output_buf);
					mutex_unlock(&isp->isp3a_lock);

					isp->isp3a_stat_ready = false;
					if (ret != sh_css_success)
						v4l2_err(&atomisp_dev,
							"get 3a statistics"
							" failed, not "
							"enough memory.\n");
				}
			}
		} while (!(irq_infos & SH_CSS_IRQ_INFO_FRAME_DONE));

		mutex_unlock(&isp->isp_lock);

		/*
		 * we check whether invalid_frame is set, if so, we do not
		 * return the buffer to the user-space, but uses it again
		 */
		if (!timeout_flag && !isp->sw_contex.invalid_frame) {
			if (vb_capture) {
				isp->frame_status[vb_capture->i] = fr_status;
				do_gettimeofday(&vb_capture->ts);
				vb_capture->field_count++;
				/*mark videobuffer done for dequeue*/
				vb_capture->state = VIDEOBUF_DONE;
			}

			if (vb_preview) {
				do_gettimeofday(&vb_preview->ts);
				vb_preview->field_count++;
				/*mark videobuffer done for dequeue*/
				vb_preview->state = VIDEOBUF_DONE;
			}

			/*
			 * Frame capture done, wake up any process block on
			 * current active buffer
			 */
			if (vb_preview)
				wake_up(&vb_preview->done);
			if (vb_capture)
				wake_up(&vb_capture->done);

			vb_preview = NULL;
			vb_capture = NULL;
		}

		if (!timeout_flag)
			timeout_cnt = 0;
	}

error:
	if (isp->isp_subdev.video_out_vf.opened)
		isp->vf_frame = NULL;

	isp->sw_contex.error = true;
	timeout_cnt = 0;

	if (vb_preview)
		vb_preview->state = VIDEOBUF_ERROR;
	if (vb_capture)
		vb_capture->state = VIDEOBUF_ERROR;

	/*
	 * Frame capture error, wake up any process block on current
	 * active buffer
	 */
	if (vb_preview)
		wake_up(&vb_preview->done);
	if (vb_capture)
		wake_up(&vb_capture->done);
	return;
}

/*
 * utils for buffer allocation/free
 */
#define bytes_to_pgnr_ceil(bytes) \
    (((bytes) + ((1<<PAGE_SHIFT) - 1)) >> PAGE_SHIFT)

int atomisp_get_frame_pgnr(const struct sh_css_frame *frame, u32 * p_pgnr)
{
	if (!frame) {
		v4l2_err(&atomisp_dev,
			    "%s: NULL frame pointer ERROR.\n",
			    __func__);
		return -EINVAL;
	}

	(*p_pgnr) = bytes_to_pgnr_ceil(frame->data_bytes);
	return 0;
}

/*
 * Get internal fmt according to V4L2 fmt
 */
enum sh_css_frame_format v4l2_fmt_to_sh_fmt(u32 fmt)
{
	switch (fmt) {
	case V4L2_PIX_FMT_YUV420:
		return SH_CSS_FRAME_FORMAT_YUV420;
	case V4L2_PIX_FMT_YVU420:
		return SH_CSS_FRAME_FORMAT_YV12;
	case V4L2_PIX_FMT_YUV422P:
		return SH_CSS_FRAME_FORMAT_YUV422;
	case V4L2_PIX_FMT_YUV444:
		return SH_CSS_FRAME_FORMAT_YUV444;
	case V4L2_PIX_FMT_NV12:
		return SH_CSS_FRAME_FORMAT_NV12;
	case V4L2_PIX_FMT_NV21:
		return SH_CSS_FRAME_FORMAT_NV21;
	case V4L2_PIX_FMT_NV16:
		return SH_CSS_FRAME_FORMAT_NV16;
	case V4L2_PIX_FMT_NV61:
		return SH_CSS_FRAME_FORMAT_NV61;
	case V4L2_PIX_FMT_UYVY:
		return SH_CSS_FRAME_FORMAT_UYVY;
	case V4L2_PIX_FMT_YUYV:
		return SH_CSS_FRAME_FORMAT_YUYV;
	case V4L2_PIX_FMT_RGB24:
		return SH_CSS_FRAME_FORMAT_PLANAR_RGB888;
	case V4L2_PIX_FMT_RGB32:
		return SH_CSS_FRAME_FORMAT_RGBA888;
	case V4L2_PIX_FMT_RGB565:
		return SH_CSS_FRAME_FORMAT_RGB565;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return SH_CSS_FRAME_FORMAT_RAW;
	default:
		return -EINVAL;
	}
}
/*
 * raw format match between SH format and V4L2 format
 */
static int raw_output_format_match_input(u32 input, u32 output)
{
	if ((input == SH_CSS_INPUT_FORMAT_RAW_12) &&
	    ((output == V4L2_PIX_FMT_SRGGB12) ||
	     (output == V4L2_PIX_FMT_SGRBG12) ||
	     (output == V4L2_PIX_FMT_SBGGR12) ||
	     (output == V4L2_PIX_FMT_SGBRG12)))
		return 0;

	if ((input == SH_CSS_INPUT_FORMAT_RAW_10) &&
	    ((output == V4L2_PIX_FMT_SRGGB10) ||
	     (output == V4L2_PIX_FMT_SGRBG10) ||
	     (output == V4L2_PIX_FMT_SBGGR10) ||
	     (output == V4L2_PIX_FMT_SGBRG10)))
		return 0;

	if ((input == SH_CSS_INPUT_FORMAT_RAW_8) &&
	    ((output == V4L2_PIX_FMT_SRGGB8) ||
	     (output == V4L2_PIX_FMT_SGRBG8) ||
	     (output == V4L2_PIX_FMT_SBGGR8) ||
	     (output == V4L2_PIX_FMT_SGBRG8)))
		return 0;

	if ((input == SH_CSS_INPUT_FORMAT_RAW_16) &&
	    (output == V4L2_PIX_FMT_SBGGR16))
		return 0;

	return -EINVAL;
}
/*
 * 2 types of format: SH format, v4l2 format
 * atomisp_format_bridge is a wrapper format for the other 2
 */
struct atomisp_format_bridge *get_atomisp_format_bridge(
					unsigned int pixelformat)
{
	u32 i;
	for (i = 0; i < atomisp_output_fmts_num; i++) {
		if (atomisp_output_fmts[i].pixelformat == pixelformat)
			return (struct atomisp_format_bridge *)
					(&(atomisp_output_fmts[i]));
	}
	return NULL;
}

struct atomisp_format_bridge *get_atomisp_format_bridge_from_mbus(
				enum v4l2_mbus_pixelcode mbus_code)
{
	int i;
	for (i = 0; i < atomisp_output_fmts_num; i++) {
		if (atomisp_output_fmts[i].mbus_code == mbus_code)
			return (struct atomisp_format_bridge *)
				(&(atomisp_output_fmts[i]));
	}
	return NULL;
}
static u32 get_pixel_depth(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YVU420:
		return 12;
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		return 16;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_YUV444:
		return 24;
	case V4L2_PIX_FMT_RGB32:
		return 32;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return 8;
	default:
		return 8 * 2;	/* raw type now */
	}
}

int is_pixelformat_raw(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return 1;
	default:
		return 0;
	}
}

static int is_mbusformat_raw(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_MBUS_FMT_SBGGR8_1X8:
	case V4L2_MBUS_FMT_SGRBG8_1X8:
	case V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_BE:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_BE:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_LE:
	case V4L2_MBUS_FMT_SBGGR10_1X10:
	case V4L2_MBUS_FMT_SGBRG10_1X10:
	case V4L2_MBUS_FMT_SGRBG10_1X10:
	case V4L2_MBUS_FMT_SRGGB10_1X10:
	case V4L2_MBUS_FMT_SBGGR12_1X12:
		return 1;
	default:
		return 0;
	}
}

static int get_sh_input_format(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_YUV420:
		return SH_CSS_INPUT_FORMAT_YUV420_8;

	case V4L2_PIX_FMT_YUV422P:
		return SH_CSS_INPUT_FORMAT_YUV422_8;

	case V4L2_PIX_FMT_RGB565:
		return SH_CSS_INPUT_FORMAT_RGB_565;

	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return SH_CSS_INPUT_FORMAT_RAW_8;

	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		return SH_CSS_INPUT_FORMAT_RAW_10;

	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		return SH_CSS_INPUT_FORMAT_RAW_12;

	case V4L2_PIX_FMT_SBGGR16:
		return SH_CSS_INPUT_FORMAT_RAW_16;

	default:
		return -EINVAL;
	}
}

/* return whether v4l2 format is supported */
int atomisp_is_pixelformat_supported(u32 pixelformat)
{
	int i;

	for (i = 0; i < atomisp_output_fmts_num; i++) {
		if (pixelformat == atomisp_output_fmts[i].pixelformat)
			return 1;
	}
	return 0;
}
/*
 * for different isp run mode: preview/still/video.
 * return which mode support viewfinder output
 * still/video support dual stream output
 */
bool atomisp_is_viewfinder_support(struct atomisp_device *isp)
{
	if (isp->sw_contex.run_mode == CI_MODE_PREVIEW)
		return false;

	if ((isp->sw_contex.run_mode == CI_MODE_STILL_CAPTURE) &&
	    (isp->main_format->out_sh_fmt == SH_CSS_FRAME_FORMAT_RAW) &&
	    (isp->sw_contex.bypass))
		return false;

	if (!isp->isp_subdev.video_out_vf.opened)
		return false;

	return true;
}

/*
 * ISP features control function
 */

/*
 * Set ISP capture mode based on current settings
 */
static void atomisp_update_capture_mode(struct atomisp_device *isp)
{
	if (isp->params.low_light) {
		sh_css_capture_set_mode(SH_CSS_CAPTURE_MODE_LOW_LIGHT);
	} else if (isp->params.gdc_cac_en || isp->params.macc_en) {
		sh_css_capture_set_mode(SH_CSS_CAPTURE_MODE_ADVANCED);
	} else {
		sh_css_capture_set_mode(SH_CSS_CAPTURE_MODE_PRIMARY);
	}
}

/*
 * Function to enable/disable lens geometry distortion correction (GDC) and
 * chromatic aberration correction (CAC)
 */
int atomisp_gdc_cac(struct atomisp_device *isp, int flag, __s32 * value)
{
	struct sh_css_morph_table *tab;

	if (flag == 0) {
		*value = isp->params.gdc_cac_en;
		return 0;
	}

	isp->params.gdc_cac_en = (*value == 0) ? 0 : 1;
	if (isp->params.gdc_cac_en) {
		tab = isp->inputs[isp->input_curr].morph_table;
		if (tab) {
			sh_css_morph_table_free(tab);
			isp->inputs[isp->input_curr].morph_table = NULL;
		}
	} else {
		sh_css_set_morph_table(NULL);
	}
	atomisp_update_capture_mode(isp);
	return 0;
}

/*
 * Function to enable/disable low light mode including ANR
 */
int atomisp_low_light(struct atomisp_device *isp, int flag, __s32 * value)
{
	if (flag == 0) {
		*value = isp->params.low_light;
		return 0;
	}

	isp->params.low_light = (*value != 0);
	atomisp_update_capture_mode(isp);
	return 0;
}

/*
 * Function to enable/disable extra nosie reduction (XNR) in low light
 * condition
 */
int atomisp_xnr(struct atomisp_device *isp, int flag, int *arg)
{
	int xnr_enable = (*arg == 0) ? 0 : 1;

	if (flag == 0) {
		*arg = isp->params.xnr_en;
		return 0;
	}

	sh_css_capture_enable_xnr(xnr_enable);

	return 0;
}

/*
 * Function to configure bayer nosie reduction
 */
int atomisp_bayer_nr(struct atomisp_device *isp, int flag,
			     void *config)
{
	struct atomisp_nr_config *arg = (struct atomisp_nr_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.nr_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	/* Get nr config from current setup */
	if (flag == 0) {
		memcpy(arg, &isp->params.nr_config, sizeof(*arg));
	} else {
		/* Set nr config to isp parameters */
		memcpy(&isp->params.nr_config, arg,
			sizeof(struct sh_css_nr_config));
		sh_css_set_nr_config(&isp->params.nr_config);
	}
	return 0;
}

/*
 * Function to configure temporal nosie reduction (TNR)
 */
int atomisp_tnr(struct atomisp_device *isp, int flag,
			void *config)
{
	struct atomisp_tnr_config *arg = (struct atomisp_tnr_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.tnr_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	/* Get tnr config from current setup */
	if (flag == 0) {
		/* Get tnr config from current setup */
		memcpy(arg, &isp->params.tnr_config, sizeof(*arg));
	} else {
		/* Set tnr config to isp parameters */
		memcpy(&isp->params.tnr_config, arg,
			sizeof(struct sh_css_tnr_config));
		sh_css_set_tnr_config(&isp->params.tnr_config);
	}

	return 0;
}

/*
 * Function to get histogram data for image frame
 */
int atomisp_histogram(struct atomisp_device *isp, int flag, void *config)
{
	struct atomisp_histogram *arg = (struct atomisp_histogram *)config;
	struct sh_css_histogram *histogram;
	int ret = 0;
	unsigned int *buffer;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(*histogram)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		buffer = kzalloc(2048 * sizeof(unsigned int), GFP_KERNEL);
		if (buffer == NULL) {
			v4l2_err(&atomisp_dev,
					"buffer allocate error\n");
			return -ENOMEM;
		}

		ret = sh_css_histogram_allocate(2048, &histogram);
		if (ret != sh_css_success) {
			v4l2_err(&atomisp_dev,
					"sh_css_histogram_allocate failed\n");
			goto buffer_free;
		}

		if (isp->vf_frame == NULL) {
			v4l2_err(&atomisp_dev,
					"No frame for histogram\n");
			ret = -EINVAL;
			goto histogram_free;
		}

		ret = sh_css_histogram_start(isp->vf_frame, histogram);
		if (ret != sh_css_success) {
			v4l2_err(&atomisp_dev,
					"sh_css_get_y_histogram failed\n");
			goto histogram_free;
		}
		sh_css_wait_for_completion();

		ret = hmm_load(histogram->data, buffer,
			histogram->num_elements * sizeof(unsigned int));
		if (ret) {
			v4l2_err(&atomisp_dev, "hmm_load failed\n");
			goto histogram_free;
		}

		ret = copy_to_user(arg->data, buffer,
			histogram->num_elements * sizeof(unsigned int));
		if (ret) {
			v4l2_err(&atomisp_dev,
					"copy to user failed\n");
			ret = -EFAULT;
			goto histogram_free;
		}

		ret = 0;
		arg->num_elements = histogram->num_elements;

histogram_free:
		sh_css_histogram_free(histogram);
buffer_free:
		kfree(buffer);

		return ret;
	}

	isp->params.histogram_elenum = arg->num_elements;

	return 0;
}

/*
 * Function to configure black level compensation
 */
int atomisp_black_level(struct atomisp_device *isp, int flag,
				void *config)
{
	struct atomisp_ob_config *arg = (struct atomisp_ob_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.ob_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get ob config from current setup */
		const struct sh_css_ob_config *ob_config;
		sh_css_get_ob_config(&ob_config);
		memcpy(arg, ob_config, sizeof(*arg));
	} else {
		/* Set ob config to isp parameters */
		memcpy(&isp->params.ob_config, arg,
			sizeof(struct sh_css_ob_config));
		sh_css_set_ob_config(&isp->params.ob_config);
	}

	return 0;
}

/*
 * Function to configure Ycc nosie reduction
 */
int atomisp_ycc_nr(struct atomisp_device *isp, int flag,
			   void *config)
{
	struct atomisp_nr_config *arg = (struct atomisp_nr_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.nr_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get nr config from current setup */
		const struct sh_css_nr_config *nr_config;
		sh_css_get_nr_config(&nr_config);
		memcpy(arg, nr_config, sizeof(*arg));
	} else {
		/* Set nr config to isp parameters */
		memcpy(&isp->params.nr_config, arg,
			sizeof(isp->params.nr_config));
		sh_css_set_nr_config(&isp->params.nr_config);
	}

	return 0;
}

/*
 * Function to configure edge enhancement
 */
int atomisp_ee(struct atomisp_device *isp, int flag,
		       void *config)
{
	struct atomisp_ee_config *arg = (struct atomisp_ee_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.ee_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get ee config from current setup */
		const struct sh_css_ee_config *ee_config;
		sh_css_get_ee_config(&ee_config);
		memcpy(arg, ee_config, sizeof(*arg));
	} else {
		/* Set ee config to isp parameters */
		memcpy(&isp->params.ee_config, arg,
		       sizeof(isp->params.ee_config));
		sh_css_set_ee_config(&isp->params.ee_config);
	}

	return 0;
}

/*
 * Function to update Gamma table for gamma, brightness and contrast config
 */
int atomisp_gamma(struct atomisp_device *isp, int flag,
			  void *config)
{
	struct atomisp_gamma_table *arg = (struct atomisp_gamma_table *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.gamma_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get gamma table from current setup */
		const struct sh_css_gamma_table *tab;
		sh_css_get_gamma_table(&tab);
		memcpy(arg, tab, sizeof(*arg));
	} else {
		/* Set gamma table to isp parameters */
		memcpy(&isp->params.gamma_table, arg,
		       sizeof(isp->params.gamma_table));
		sh_css_set_gamma_table(&isp->params.gamma_table);
	}

	return 0;
}

/*
 * Function to update Ctc table for Chroma Enhancement
 */
int atomisp_ctc(struct atomisp_device *isp, int flag,
			  void *config)
{
	struct atomisp_ctc_table *arg = (struct atomisp_ctc_table *) config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.ctc_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get ctc table from current setup */
		const struct sh_css_ctc_table *tab;
		sh_css_get_ctc_table(&tab);
		memcpy(arg, tab, sizeof(*arg));
	} else {
		/* Set gamma table to isp parameters */
		memcpy(&isp->params.ctc_table, arg,
			sizeof(isp->params.ctc_table));
		sh_css_set_ctc_table(&isp->params.ctc_table);
	}

	return 0;
}

void atomisp_free_internal_buffers(struct atomisp_device *isp)
{
	struct sh_css_morph_table *tab;

	tab = isp->inputs[isp->input_curr].morph_table;
	if (tab) {
		sh_css_morph_table_free(tab);
		isp->inputs[isp->input_curr].morph_table = NULL;
	}

	if (isp->params.vf_overlay) {
		if (isp->params.vf_overlay->frame)
			sh_css_frame_free(isp->params.vf_overlay->frame);
		kfree(isp->params.vf_overlay);
		isp->params.vf_overlay = NULL;
	}

	if (isp->raw_output_frame) {
		sh_css_frame_free(isp->raw_output_frame);
		isp->raw_output_frame = NULL;
	}
}

void atomisp_free_3a_buffers(struct atomisp_device *isp)
{
	/* 3A statistics use vmalloc, DIS use kmalloc */
	if (isp->params.s3a_output_buf)
		vfree(isp->params.s3a_output_buf);
	isp->params.s3a_output_buf = NULL;
	isp->params.s3a_output_bytes = 0;
}

void atomisp_free_dis_buffers(struct atomisp_device *isp)
{
	if (isp->params.dis_hor_proj_buf)
		kfree(isp->params.dis_hor_proj_buf);
	if (isp->params.dis_ver_proj_buf)
		kfree(isp->params.dis_ver_proj_buf);
	if (isp->params.dis_hor_coef_buf)
		kfree(isp->params.dis_hor_coef_buf);
	if (isp->params.dis_ver_coef_buf)
		kfree(isp->params.dis_ver_coef_buf);
	isp->params.dis_hor_proj_buf = NULL;
	isp->params.dis_ver_proj_buf = NULL;
	isp->params.dis_hor_coef_buf = NULL;
	isp->params.dis_ver_coef_buf = NULL;
	isp->params.dis_hor_proj_bytes = 0;
	isp->params.dis_ver_proj_bytes = 0;
	isp->params.dis_hor_coef_bytes = 0;
	isp->params.dis_ver_coef_bytes = 0;
}

static void atomisp_update_grid_info(struct atomisp_device *isp)
{
	struct sh_css_grid_info old_info = isp->params.curr_grid_info;

	switch (isp->sw_contex.run_mode) {
	case CI_MODE_PREVIEW:
		sh_css_preview_get_grid_info(&isp->params.curr_grid_info);
		break;
	case CI_MODE_VIDEO:
		sh_css_video_get_grid_info(&isp->params.curr_grid_info);
		break;
	default:
		sh_css_capture_get_grid_info(&isp->params.curr_grid_info);
		break;
	}
	/* If the grid info has changed, we need to reallocate
	   the buffers for 3A and DIS statistics. */
	if (memcmp(&old_info, &isp->params.curr_grid_info, sizeof(old_info)) ||
	    !isp->params.s3a_output_buf || !isp->params.dis_hor_coef_buf) {
		/* We must free all buffers because they no longer match
		   the grid size. */
		atomisp_free_3a_buffers(isp);
		atomisp_free_dis_buffers(isp);
		/* 3A statistics. These can be big, so we use vmalloc. */
		isp->params.s3a_output_bytes =
					isp->params.curr_grid_info.s3a_width *
					isp->params.curr_grid_info.s3a_height *
					sizeof(*isp->params.s3a_output_buf);

		isp->params.s3a_output_buf = vmalloc(
				isp->params.s3a_output_bytes);

		if (isp->params.s3a_output_buf == NULL)
			goto err_3a;

		/* DIS coefficients. */
		isp->params.dis_hor_coef_bytes =
				isp->params.curr_grid_info.dis_hor_coef_num *
				SH_CSS_DIS_NUM_COEF_TYPES *
				sizeof(*isp->params.dis_hor_coef_buf);

		isp->params.dis_ver_coef_bytes =
				isp->params.curr_grid_info.dis_ver_coef_num *
				SH_CSS_DIS_NUM_COEF_TYPES *
				sizeof(*isp->params.dis_ver_coef_buf);

		isp->params.dis_hor_coef_buf =
			kzalloc(isp->params.dis_hor_coef_bytes, GFP_KERNEL);
		if (isp->params.dis_hor_coef_buf == NULL)
			goto err_dis;

		isp->params.dis_ver_coef_buf =
			kzalloc(isp->params.dis_ver_coef_bytes, GFP_KERNEL);
		if (isp->params.dis_ver_coef_buf == NULL)
			goto err_dis;

		/* DIS projections. */
		isp->params.dis_hor_proj_bytes =
				isp->params.curr_grid_info.dis_height *
				SH_CSS_DIS_NUM_COEF_TYPES *
				sizeof(*isp->params.dis_hor_proj_buf);

		isp->params.dis_ver_proj_bytes =
				isp->params.curr_grid_info.dis_width *
				SH_CSS_DIS_NUM_COEF_TYPES *
				sizeof(*isp->params.dis_ver_proj_buf);

		isp->params.dis_hor_proj_buf =
			kzalloc(isp->params.dis_hor_proj_bytes, GFP_KERNEL);
		if (isp->params.dis_hor_proj_buf == NULL)
			goto err_dis;

		isp->params.dis_ver_proj_buf =
			kzalloc(isp->params.dis_ver_proj_bytes, GFP_KERNEL);
		if (isp->params.dis_ver_proj_buf == NULL)
			goto err_dis;
	}
	return;
	/* Failure for 3A buffers does not influence DIS buffers */
err_3a:
	v4l2_err(&atomisp_dev,
		    "Failed allocate memory for 3A statistics\n");
	atomisp_free_3a_buffers(isp);
	return;
err_dis:
	v4l2_err(&atomisp_dev,
		    "Failed allocate memory for DIS statistics\n");
	atomisp_free_dis_buffers(isp);

}

static void atomisp_curr_user_grid_info(struct atomisp_device *isp,
				    struct atomisp_grid_info_user *info)
{
	info->s3a_width             = isp->params.curr_grid_info.s3a_width;
	info->s3a_height            = isp->params.curr_grid_info.s3a_height;
	info->s3a_bqs_per_grid_cell =
	    isp->params.curr_grid_info.s3a_bqs_per_grid_cell;
	info->dis_width             = isp->params.curr_grid_info.dis_width;
	info->dis_height            = isp->params.curr_grid_info.dis_height;
	info->dis_bqs_per_grid_cell =
	    isp->params.curr_grid_info.dis_bqs_per_grid_cell;
	info->dis_hor_coef_num      =
	    isp->params.curr_grid_info.dis_hor_coef_num;
	info->dis_ver_coef_num      =
	    isp->params.curr_grid_info.dis_ver_coef_num;
}

/*
 * Function to update Gdc table for gdc
 */
int atomisp_gdc_cac_table(struct atomisp_device *isp, int flag,
	void *config)
{
	int ret;
	int i;
	struct atomisp_morph_table *arg = (struct atomisp_morph_table *)config;

	if (flag == 0) {
		/* Get gamma table from current setup */
		const struct sh_css_morph_table *tab;
		sh_css_get_morph_table(&tab);

		arg->width = tab->width;
		arg->height = tab->height;

		for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			ret = copy_to_user(arg->coordinates_x[i],
				tab->coordinates_x[i], tab->height *
				tab->width * sizeof(*tab->coordinates_x[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
					"Failed to copy to User for x\n");
				return -EFAULT;
			}
			ret = copy_to_user(arg->coordinates_y[i],
				tab->coordinates_y[i], tab->height *
				tab->width * sizeof(*tab->coordinates_y[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
					"Failed to copy to User for y\n");
				return -EFAULT;
			}
		}
	} else {
		struct sh_css_morph_table *tab;
		tab = isp->inputs[isp->input_curr].morph_table;
		/* free first if we have one */
		if (tab) {
			sh_css_morph_table_free(tab);
			isp->inputs[isp->input_curr].morph_table = NULL;
		}

		/* allocate new one */
		tab = sh_css_morph_table_allocate(arg->width, arg->height);

		if (!tab) {
			v4l2_err(&atomisp_dev, "out of memory\n");
			return -EINVAL;
		}

		for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			ret = copy_from_user(tab->coordinates_x[i],
				(void __user *)arg->coordinates_x[i],
				arg->height * arg->width *
				sizeof(*arg->coordinates_x[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
				"Failed to copy from User for x, ret %d\n",
				ret);
				sh_css_morph_table_free(tab);
				return -EFAULT;
			}
			ret = copy_from_user(tab->coordinates_y[i],
				(void __user *)arg->coordinates_y[i],
				arg->height * arg->width *
				sizeof(*arg->coordinates_y[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
				"Failed to copy from User for y, ret is %d\n",
				ret);
				sh_css_morph_table_free(tab);
				return -EFAULT;
			}
		}
		isp->inputs[isp->input_curr].morph_table = tab;
		sh_css_set_morph_table(tab);
	}

	return 0;
}

int atomisp_macc_table(struct atomisp_device *isp, int flag,
	void *config)
{
	struct sh_css_macc_table *macc_table;
	struct atomisp_macc_config *arg = (struct atomisp_macc_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(arg->table) != sizeof(*macc_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	switch (arg->color_effect) {
	case V4L2_COLORFX_SKY_BLUE:
		macc_table = &blue_macc_table;
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		macc_table = &green_macc_table;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_LOW:
		macc_table = &skin_low_macc_table;
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		macc_table = &skin_medium_macc_table;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_HIGH:
		macc_table = &skin_high_macc_table;
		break;
	default:
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get macc table from current setup */
		memcpy(&arg->table, macc_table,
		       sizeof(struct sh_css_macc_table));
	} else {
		memcpy(macc_table, &arg->table,
		       sizeof(struct sh_css_macc_table));
	}

	return 0;
}

int atomisp_dis_vector(struct atomisp_device *isp,
		       void *config)
{
	struct atomisp_dis_config *arg = (struct atomisp_dis_config *)config;

	/* The dis parameter is initialized at start_video_capture
	 * in atomisp_work
	 */
	mutex_lock(&isp->isp_lock);
	isp->params.dis_x = arg->dis_x;
	isp->params.dis_y = arg->dis_y;
	mutex_unlock(&isp->isp_lock);
	return 0;
}

/*
 * Function to set/get image stablization statistics
 */
int atomisp_dis_stat(struct atomisp_device *isp, int flag,
		     void *config)
{
	int error;
	struct atomisp_dis_config *arg = (struct atomisp_dis_config *)config;

	if (flag == 0) {
		long time_left;

		if (arg->w_sdis_vertproj_tbl == NULL ||
		    arg->w_sdis_horiproj_tbl == NULL ||
		    isp->params.dis_hor_proj_buf    == NULL ||
		    isp->params.dis_ver_proj_buf    == NULL)
			return -EINVAL;

		/* isp need to be streaming to get DIS statistics */
		if (!isp->sw_contex.isp_streaming)
			return -EINVAL;
		if (!isp->params.video_dis_en)
			return -EINVAL;

		INIT_COMPLETION(isp->dis_state_complete);

		time_left =
		    wait_for_completion_timeout(&isp->dis_state_complete,
						1 * HZ);

		/* Timeout to get the statistics */
		if (time_left == 0) {
			v4l2_err(&atomisp_dev,
				 "Failed to wait frame DIS state\n");
			return -EINVAL;
		}


		sh_css_get_dis_projections(isp->params.dis_hor_proj_buf,
					   isp->params.dis_ver_proj_buf);

		error = copy_to_user(arg->w_sdis_vertproj_tbl,
				     isp->params.dis_ver_proj_buf,
				     isp->params.dis_ver_proj_bytes);
		if (error)
			return -EFAULT;

		error = copy_to_user(arg->w_sdis_horiproj_tbl,
				     isp->params.dis_hor_proj_buf,
				     isp->params.dis_hor_proj_bytes);
		if (error)
			return -EFAULT;
	} else {
		if (arg->sdis_vertcoef_tbl == NULL ||
		    arg->sdis_horicoef_tbl == NULL ||
		    isp->params.dis_hor_coef_buf  == NULL ||
		    isp->params.dis_ver_coef_buf  == NULL)
			return -EINVAL;

		error = copy_from_user(isp->params.dis_hor_coef_buf,
				       (void __user *)arg->sdis_horicoef_tbl,
				       isp->params.dis_hor_coef_bytes);
		if (error)
			return -EFAULT;
		error = copy_from_user(isp->params.dis_ver_coef_buf,
				       (void __user *)arg->sdis_vertcoef_tbl,
				       isp->params.dis_ver_coef_bytes);
		if (error)
			return -EFAULT;
		sh_css_set_dis_coefficients(isp->params.dis_hor_coef_buf,
					    isp->params.dis_ver_coef_buf);
	}
	return 0;
}

/*
 * Function to set/get 3A stat from isp
 */
int atomisp_3a_stat(struct atomisp_device *isp, int flag,
		    void *config)
{
	struct atomisp_3a_statistics *arg =
	    (struct atomisp_3a_statistics *)config;
	unsigned long ret;

	if (flag != 0)
		return -EINVAL;

	if (arg == NULL)
		return -EINVAL;

	/* sanity check to avoid writing into unallocated memory. */
	if (isp->params.s3a_output_bytes == 0)
		return -EINVAL;

	/* If the grid info in the argument differs from the current
	   grid info, we tell the caller to reset the grid size and
	   try again. */
	if (memcmp(&arg->grid_info, &isp->params.curr_grid_info,
		   sizeof(isp->params.curr_grid_info)) != 0)
		return -EAGAIN;

	mutex_lock(&isp->isp3a_lock);
	ret = copy_to_user(arg->data, isp->params.s3a_output_buf,
			   isp->params.s3a_output_bytes);
	mutex_unlock(&isp->isp3a_lock);
	if (ret) {
		v4l2_err(&atomisp_dev,
			    "copy to user failed: copied %lu bytes\n", ret);
		return -EFAULT;
	}
	return 0;
}

/*
 * Function to set/get isp parameters to isp
 */
int atomisp_param(struct atomisp_device *isp, int flag,
			  void *config)
{
	struct atomisp_parm *arg = (struct atomisp_parm *)config;

	/* Read parameter for 3A bianry info */
	if (flag == 0) {
		if (&arg->info == NULL) {
			v4l2_err(&atomisp_dev,
				    "ERROR: NULL pointer in grid_info\n");
			return -EINVAL;
		}
		atomisp_curr_user_grid_info(isp, &arg->info);
		return 0;
	}

	if (sizeof(arg->wb_config) != sizeof(isp->params.wb_config))
		goto INVALID_PARM;
	if (sizeof(arg->cc_config) != sizeof(isp->params.cc_config))
		goto INVALID_PARM;
	if (sizeof(arg->ob_config) != sizeof(isp->params.ob_config))
		goto INVALID_PARM;
	if (sizeof(arg->de_config) != sizeof(isp->params.de_config))
		goto INVALID_PARM;
	if (sizeof(arg->dp_config) != sizeof(isp->params.dp_config))
		goto INVALID_PARM;
	if (sizeof(arg->nr_config) != sizeof(isp->params.nr_config))
		goto INVALID_PARM;
	if (sizeof(arg->ee_config) != sizeof(isp->params.ee_config))
		goto INVALID_PARM;
	if (sizeof(arg->tnr_config) != sizeof(isp->params.tnr_config))
		goto INVALID_PARM;

	memcpy(&isp->params.wb_config, &arg->wb_config,
	       sizeof(struct sh_css_wb_config));
	memcpy(&isp->params.ob_config, &arg->ob_config,
	       sizeof(struct sh_css_ob_config));
	memcpy(&isp->params.dp_config, &arg->dp_config,
	       sizeof(struct sh_css_dp_config));
	memcpy(&isp->params.de_config, &arg->de_config,
	       sizeof(struct sh_css_de_config));
	memcpy(&isp->params.nr_config, &arg->nr_config,
	       sizeof(struct sh_css_nr_config));
	memcpy(&isp->params.ee_config, &arg->ee_config,
	       sizeof(struct sh_css_ee_config));
	memcpy(&isp->params.tnr_config, &arg->tnr_config,
	       sizeof(struct sh_css_tnr_config));

	if (isp->params.color_effect == V4L2_COLORFX_NEGATIVE) {
		arg->cc_config.matrix[3] = -arg->cc_config.matrix[3];
		arg->cc_config.matrix[4] = -arg->cc_config.matrix[4];
		arg->cc_config.matrix[5] = -arg->cc_config.matrix[5];
		arg->cc_config.matrix[6] = -arg->cc_config.matrix[6];
		arg->cc_config.matrix[7] = -arg->cc_config.matrix[7];
		arg->cc_config.matrix[8] = -arg->cc_config.matrix[8];
	}

	if (isp->params.color_effect != V4L2_COLORFX_SEPIA &&
	    isp->params.color_effect != V4L2_COLORFX_BW) {
		memcpy(&isp->params.cc_config, &arg->cc_config,
		       sizeof(struct sh_css_cc_config));
		sh_css_set_cc_config(&isp->params.cc_config);
	}

	sh_css_set_wb_config(&isp->params.wb_config);
	sh_css_set_ob_config(&isp->params.ob_config);
	sh_css_set_de_config(&isp->params.de_config);
	sh_css_set_dp_config(&isp->params.dp_config);
	sh_css_set_nr_config(&isp->params.nr_config);
	sh_css_set_ee_config(&isp->params.ee_config);
	sh_css_set_tnr_config(&isp->params.tnr_config);

	return 0;

INVALID_PARM:
	v4l2_err(&atomisp_dev,
		"%s: incompatible param.\n", __func__);
	return -EINVAL;
}

/*
 * Function to configure color effect of the image
 */
int atomisp_color_effect(struct atomisp_device *isp, int flag, __s32 *effect)
{
	const struct sh_css_cc_config *cc_config = NULL;
	const struct sh_css_macc_table *macc_table = NULL;
	const struct sh_css_ctc_table *ctc_table = NULL;

	if (flag == 0) {
		*effect = isp->params.color_effect;
		return 0;
	}

	if (*effect == isp->params.color_effect)
		return 0;

	switch (*effect) {
	case V4L2_COLORFX_NONE:
		cc_config  = isp->params.default_cc_config;
		macc_table = isp->params.default_macc_table;
		ctc_table  = isp->params.default_ctc_table;
		isp->params.macc_en = false;
		break;
	case V4L2_COLORFX_SEPIA:
		cc_config = &sepia_cc_config;
		break;
	case V4L2_COLORFX_NEGATIVE:
		cc_config = &nega_cc_config;
		break;
	case V4L2_COLORFX_BW:
		cc_config = &mono_cc_config;
		break;
	case V4L2_COLORFX_SKY_BLUE:
		macc_table = &blue_macc_table;
		isp->params.macc_en = true;
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		macc_table = &green_macc_table;
		isp->params.macc_en = true;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_LOW:
		macc_table = &skin_low_macc_table;
		isp->params.macc_en = true;
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		macc_table = &skin_medium_macc_table;
		isp->params.macc_en = true;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_HIGH:
		macc_table = &skin_high_macc_table;
		isp->params.macc_en = true;
		break;
	case V4L2_COLORFX_VIVID:
		ctc_table = &vivid_ctc_table;
		break;
	default:
		return -EINVAL;
	}
	atomisp_update_capture_mode(isp);

	if (cc_config)
		sh_css_set_cc_config(cc_config);
	if (macc_table)
		sh_css_set_macc_table(macc_table);
	if (ctc_table)
		sh_css_set_ctc_table(ctc_table);
	isp->params.color_effect = (u32)*effect;
	return 0;
}

/*
 * Function to configure bad pixel correction
 */
int atomisp_bad_pixel(struct atomisp_device *isp, int flag, __s32 *value)
{

	if (flag == 0) {
		*value = isp->params.bad_pixel_en;
		return 0;
	}
	isp->params.bad_pixel_en = (*value == 0) ? 0 : 1;

	return 0;
}

/*
 * Function to configure bad pixel correction params
 */
int atomisp_bad_pixel_param(struct atomisp_device *isp, int flag,
	void *config)
{
	struct atomisp_dp_config *arg = (struct atomisp_dp_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.dp_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get bad pixel from current setup */
		memcpy(arg, &isp->params.dp_config, sizeof(*arg));
	} else {
		/* Set bad pixel to isp parameters */
		memcpy(&isp->params.dp_config, arg, sizeof(*arg));
		sh_css_set_dp_config(&isp->params.dp_config);
	}

	return 0;
}

/*
 * Function to enable/disable video image stablization
 */
int atomisp_video_stable(struct atomisp_device *isp, int flag, __s32 * value)
{
	if (flag == 0)
		*value = isp->params.video_dis_en;
	else
		isp->params.video_dis_en = (*value == 0) ? 0 : 1;

	return 0;
}

/*
 * Function to configure fixed pattern noise
 */
int atomisp_fixed_pattern(struct atomisp_device *isp, int flag, __s32 * value)
{

	if (flag == 0) {
		*value = isp->params.fpn_en;
		return 0;
	}

	if (*value == 0) {
		isp->params.fpn_en = 0;
		return 0;
	}

	/* Add function to get black from from sensor with shutter off */
	return 0;
}

/*
 * Function to configure fixed pattern noise table
 */
int atomisp_fixed_pattern_table(struct atomisp_device *isp, int flag,
	void *config)
{
	struct atomisp_frame *arg = (struct atomisp_frame *)config;
	struct sh_css_frame *raw_black_frame = NULL;
	struct sh_css_frame_info info;
	char *tmp_buf;
	int ret = 0;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(*raw_black_frame)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0)
		return -EINVAL;
	else {
		memcpy(&info, &arg->info, sizeof(struct sh_css_frame_info));

		tmp_buf = vmalloc(arg->data_bytes);
		if (!tmp_buf)
			return -ENOMEM;

		if (sh_css_frame_allocate_from_info(&raw_black_frame, &info)) {
			ret = -ENOMEM;
			goto failed1;
		}

		if (copy_from_user(tmp_buf, arg->data, arg->data_bytes)) {
			ret = -EFAULT;
			goto failed2;
		}

		if (hmm_store(raw_black_frame->data, tmp_buf,
			arg->data_bytes)){
			ret = -EINVAL;
			goto failed2;
		}

		sh_css_set_black_frame(raw_black_frame);
failed2:
		sh_css_frame_free(raw_black_frame);
failed1:
		vfree(tmp_buf);
	}

	return ret;
}

/*
 * Function to configure vf overlay image
 */
int atomisp_vf_overlay(struct atomisp_device *isp, int flag,
	void *config)
{
	struct atomisp_overlay *arg = (struct atomisp_overlay *)config;
	int ret = 0;
	struct sh_css_frame vf_frame;
	char *tmp_buf;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(*isp->params.vf_overlay)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (!arg->frame)
		/* from siliconhive, passing NULL means disable the feature */
		sh_css_overlay_set_for_viewfinder(NULL);

	if (copy_from_user(&vf_frame, arg->frame,
			     sizeof(struct sh_css_frame)))
		return -EFAULT;

	tmp_buf = vmalloc(vf_frame.data_bytes);
	if (!tmp_buf)
		return -ENOMEM;

	if (isp->params.vf_overlay) {
		if (isp->params.vf_overlay->frame)
			sh_css_frame_free(isp->params.vf_overlay->frame);
		kfree(isp->params.vf_overlay);
	}

	isp->params.vf_overlay = kmalloc(sizeof(struct sh_css_overlay),
							GFP_KERNEL);
	if (!isp->params.vf_overlay) {
		ret =  -ENOMEM;
		goto fail1;
	}

	if (sh_css_frame_allocate_from_info(&isp->params.vf_overlay->frame,
					   &vf_frame.info)) {
		ret = -ENOMEM;
		goto fail2;
	}

	isp->params.vf_overlay->bg_y = arg->bg_y;
	isp->params.vf_overlay->bg_u = arg->bg_u;
	isp->params.vf_overlay->bg_v = arg->bg_v;
	isp->params.vf_overlay->blend_input_perc_y = arg->blend_input_perc_y;
	isp->params.vf_overlay->blend_input_perc_u = arg->blend_input_perc_u;
	isp->params.vf_overlay->blend_input_perc_v = arg->blend_input_perc_v;
	isp->params.vf_overlay->blend_overlay_perc_y =
						arg->blend_overlay_perc_y;
	isp->params.vf_overlay->blend_overlay_perc_u =
						arg->blend_overlay_perc_u;
	isp->params.vf_overlay->blend_overlay_perc_v =
						arg->blend_overlay_perc_v;
	isp->params.vf_overlay->overlay_start_x = arg->overlay_start_x;
	isp->params.vf_overlay->overlay_start_y = arg->overlay_start_y;

	/* copy for YUV 420 */
	if (copy_from_user(tmp_buf, vf_frame.data, vf_frame.data_bytes)) {
			ret = -EFAULT;
			goto fail2;
	}

	if (hmm_store(isp->params.vf_overlay->frame->data, tmp_buf,
		vf_frame.data_bytes)){
		ret = -EINVAL;
		goto fail2;
	}

	sh_css_overlay_set_for_viewfinder(isp->params.vf_overlay);

	vfree(tmp_buf);

	return ret;

fail2:
	kfree(isp->params.vf_overlay);
fail1:
	vfree(tmp_buf);
	return ret;
}

/*
 * Function to configure false color correction
 */
int atomisp_false_color(struct atomisp_device *isp, int flag, __s32 *value)
{
	/* Get nr config from current setup */
	if (flag == 0) {
		*value = isp->params.false_color;
		return 0;
	}

	/* Set nr config to isp parameters */
	if (*value) {
		sh_css_set_de_config(isp->params.default_de_config);
	} else {
		isp->params.de_config.pixelnoise = 0;
		sh_css_set_de_config(&isp->params.de_config);
	}
	isp->params.false_color = *value;
	return 0;
}

/*
 * Function to configure bad pixel correction params
 */
int atomisp_false_color_param(struct atomisp_device *isp, int flag,
	void *config)
{
	struct atomisp_de_config *arg =
	    (struct atomisp_de_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.de_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get false color from current setup */
		memcpy(arg, &isp->params.de_config, sizeof(*arg));
	} else {
		/* Set false color to isp parameters */
		memcpy(&isp->params.de_config, arg, sizeof(*arg));
		sh_css_set_de_config(&isp->params.de_config);
	}

	return 0;
}

/*
 * Function to configure white balance params
 */
int atomisp_white_balance_param(struct atomisp_device *isp, int flag,
	void *config)
{
	struct atomisp_wb_config *arg = (struct atomisp_wb_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.wb_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get white balance from current setup */
		memcpy(arg, &isp->params.wb_config, sizeof(*arg));
	} else {
		/* Set white balance to isp parameters */
		memcpy(&isp->params.wb_config, arg, sizeof(*arg));
		sh_css_set_wb_config(&isp->params.wb_config);
	}

	return 0;
}

int atomisp_3a_config_param(struct atomisp_device *isp, int flag,
			    void *config)
{
	struct atomisp_3a_config *arg = (struct atomisp_3a_config *)config;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp->params.s3a_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get white balance from current setup */
		memcpy(arg, &isp->params.s3a_config, sizeof(*arg));
	} else {
		/* Set white balance to isp parameters */
		memcpy(&isp->params.s3a_config, arg, sizeof(*arg));
		sh_css_set_3a_config(&isp->params.s3a_config);
	}

	return 0;
}

/*
 * Function to enable/disable lens shading correction
 */
int atomisp_shading_correction(struct atomisp_device *isp, int flag,
				       __s32 *value)
{
	if (flag == 0) {
		*value = isp->params.sc_en;
		return 0;
	}

	if (*value == 0) {
		sh_css_set_shading_table(NULL);
	} else {
		sh_css_set_shading_table(
			isp->inputs[isp->input_curr].shading_table);
	}

	isp->params.sc_en = *value;

	return 0;
}

/*
 * Function to setup digital zoom
 */
int atomisp_digital_zoom(struct atomisp_device *isp, int flag, __s32 *value)
{
	u32 zoom;

	if (flag == 0) {
		sh_css_get_zoom_factor(&zoom, &zoom);
		*value = 64 - zoom;
	} else {
		if (*value < 0)
			return -EINVAL;

		zoom = *value;
		if (zoom >= 64)
			zoom = 64;
		zoom = 64 - zoom;

		sh_css_set_zoom_factor(zoom, zoom);
	}

	return 0;
}

/*
 * Function to get sensor specific info for current resolution,
 * which will be used for auto exposure conversion.
 */
int atomisp_get_sensor_mode_data(struct atomisp_device *isp,
				void *config)
{
	struct atomisp_sensor_mode_data *arg =
			(struct atomisp_sensor_mode_data *)config;
	struct camera_mipi_info *mipi_info;

	mipi_info =
	atomisp_to_sensor_mipi_info(isp->inputs[isp->input_curr].camera);
	if (mipi_info == NULL)
		return -EINVAL;

	memcpy(arg, &mipi_info->data, sizeof(*arg));
	return 0;
}

int atomisp_get_fmt(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
			    "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	memset(f, 0, sizeof(struct v4l2_format));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* VIDIOC_S_FMT already called,*/
	/* return fmt setted by app */
	if ((pipe->format) && (pipe->format->out.width != 0))
		memcpy(&f->fmt.pix, &pipe->format->out,
			sizeof(struct v4l2_pix_format));
	else {
		f->fmt.pix.width = 640;
		f->fmt.pix.height = 480;
		f->fmt.pix.pixelformat = atomisp_output_fmts[0].pixelformat;
		f->fmt.pix.bytesperline =
			get_pixel_depth(f->fmt.pix.pixelformat) *
						f->fmt.pix.width;
		f->fmt.pix.sizeimage = f->fmt.pix.height *
						f->fmt.pix.bytesperline;
		f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	}

	return 0;
}


/* This function looks up the closest available resolution. */
int atomisp_try_fmt(struct video_device *vdev, struct v4l2_format *f,
						bool *res_overflow)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct v4l2_mbus_framefmt snr_mbus_fmt;
	struct atomisp_format_bridge *fmt;
	u32 out_width = f->fmt.pix.width;
	u32 out_height = f->fmt.pix.height;
	u32 pixelformat = f->fmt.pix.pixelformat;
	u32 in_width = 0;
	u32 in_height = 0;
	int ret;
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	if (isp->inputs[isp->input_curr].camera == NULL)
		return -EINVAL;

	fmt = get_atomisp_format_bridge(pixelformat);
	if (fmt == NULL) {
		v4l2_err(&atomisp_dev, "unsupported pixelformat!\n");
		fmt = (struct atomisp_format_bridge *)&atomisp_output_fmts[0];
	}

	/* fixing me! seems tpg does not support mbus interface */
#if 0
	/*set TPG format*/
	if (isp->inputs[isp->input_curr].type == TEST_PATTERN) {
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			video, try_fmt, f);
		in_width = f->fmt.pix.width;
		in_height = f->fmt.pix.height;
		goto done;
	}
#else
	if (isp->inputs[isp->input_curr].type == TEST_PATTERN)
		return -EINVAL;
#endif
	snr_mbus_fmt.code = fmt->mbus_code;
	snr_mbus_fmt.height = out_height;
	snr_mbus_fmt.width = out_width;

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			video, try_mbus_fmt, &snr_mbus_fmt);
	if (ret)
		return ret;
	else {
		in_width = snr_mbus_fmt.width;
		in_height = snr_mbus_fmt.height;
		fmt = get_atomisp_format_bridge_from_mbus(snr_mbus_fmt.code);
		if (fmt == NULL) {
			f->fmt.pix.pixelformat = pixelformat;
			v4l2_err(&atomisp_dev, "unknown sensor format.\n");
		} else
			f->fmt.pix.pixelformat = fmt->pixelformat;
	}

done:
	v4l2_info(&atomisp_dev, "return snr_try_fmt, (wxh: %d*%d)\n",
		    in_width, in_height);
	if ((in_width < out_width) && (in_height < out_height)) {
		out_width = in_width;
		out_height = in_height;
		/* Set the flag when resolution requested is
		 * beyond the max value supported by sensor
		 */
		if (res_overflow != NULL)
			*res_overflow = true;
	}

	/* app vs isp */
	out_width = min(out_width, (u32)ATOM_ISP_MAX_WIDTH);
	out_height = min(out_height, (u32)ATOM_ISP_MAX_HEIGHT);

	out_width = max(out_width, (u32)ATOM_ISP_MIN_WIDTH);
	out_height = max(out_height, (u32)ATOM_ISP_MIN_HEIGHT);

	out_width = out_width - out_width % ATOM_ISP_STEP_WIDTH;
	out_height = out_height - out_height % ATOM_ISP_STEP_HEIGHT;

	f->fmt.pix.width = out_width;
	f->fmt.pix.height = out_height;

	return 0;
}

int atomisp_try_fmt_file(struct atomisp_device *isp, struct v4l2_format *f)
{
	u32 width = f->fmt.pix.width;
	u32 height = f->fmt.pix.height;
	u32 pixelformat = f->fmt.pix.pixelformat;
	enum v4l2_field field = f->fmt.pix.field;
	u32 depth;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	if (!atomisp_is_pixelformat_supported(pixelformat)) {
		v4l2_err(&atomisp_dev, "Wrong output pixelformat\n");
		return -EINVAL;
	}

	depth = get_pixel_depth(pixelformat);

	if (!field || field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (field != V4L2_FIELD_NONE) {
		v4l2_err(&atomisp_dev, "Wrong output field\n");
		return -EINVAL;
	}

	f->fmt.pix.field = field;

	if (width > ATOM_ISP_MAX_WIDTH)
		width = ATOM_ISP_MAX_WIDTH;
	else if (width < ATOM_ISP_MIN_WIDTH)
		width = ATOM_ISP_MIN_WIDTH;

	if (height > ATOM_ISP_MAX_HEIGHT)
		height = ATOM_ISP_MAX_HEIGHT;
	else if (height < ATOM_ISP_MIN_HEIGHT)
		height = ATOM_ISP_MIN_HEIGHT;

	width = width - width % ATOM_ISP_STEP_WIDTH;
	height = height - height % ATOM_ISP_STEP_HEIGHT;

	f->fmt.pix.width = width;
	f->fmt.pix.height = height;

	f->fmt.pix.bytesperline = (width * depth) >> 3;

	return 0;
}

static bool atomisp_input_format_is_raw(enum atomisp_input_format format)
{
	switch (format) {
	case ATOMISP_INPUT_FORMAT_RAW_6:
	case ATOMISP_INPUT_FORMAT_RAW_7:
	case ATOMISP_INPUT_FORMAT_RAW_8:
	case ATOMISP_INPUT_FORMAT_RAW_10:
	case ATOMISP_INPUT_FORMAT_RAW_12:
	case ATOMISP_INPUT_FORMAT_RAW_14:
	case ATOMISP_INPUT_FORMAT_RAW_16:
		return true;
	default:
		return false;
	}
}

static inline void
atomisp_set_sensor_mipi_to_isp(struct camera_mipi_info *mipi_info)
{
	if (mipi_info) {
		v4l2_info(&atomisp_dev,
			  "MIPI info from sensor:\n"
			  "  port: %d\n"
			  "  number of lanes: %d\n"
			  "  input format: %d\n"
			  "  bayer order: %d\n",
			  mipi_info->port,
			  mipi_info->num_lanes,
			  mipi_info->input_format,
			  mipi_info->raw_bayer_order);
		if (atomisp_input_format_is_raw(mipi_info->input_format))
			sh_css_input_set_bayer_order(
					mipi_info->raw_bayer_order);
		sh_css_input_set_format(mipi_info->input_format);
		sh_css_input_configure_port(mipi_info->port,
					    mipi_info->num_lanes,
					    0xffff4);
	} else {
		/*Use default MIPI configuration*/
		v4l2_info(&atomisp_dev,
			    "no sensor config info\n");
		sh_css_input_set_bayer_order(sh_css_bayer_order_grbg);
		sh_css_input_set_format(SH_CSS_INPUT_FORMAT_RAW_10);
		v4l2_info(&atomisp_dev, "MIPI 2 lane\n");
		sh_css_input_configure_port(SH_CSS_MIPI_PORT_4LANE,
						2, 0xffff4);
	}
}

static int atomisp_set_fmt_to_isp(struct video_device *vdev,
				   struct sh_css_frame_info *output_info,
				   struct sh_css_frame_info *raw_output_info,
				   int width, int height,
				   unsigned int pixelformat)
{
	struct camera_mipi_info *mipi_info;
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_format_bridge *format;
	int effective_input_width = pipe->format->in.width;
	int effective_input_height = pipe->format->in.height;
	int ret;

	format = get_atomisp_format_bridge(pixelformat);
	if (format == NULL)
		return -EINVAL;

	isp->main_format->out_sh_fmt = format->sh_fmt;
	pipe->format->out.pixelformat = pixelformat;

	if (isp->inputs[isp->input_curr].type != TEST_PATTERN &&
		isp->inputs[isp->input_curr].type != FILE_INPUT) {
		mipi_info = atomisp_to_sensor_mipi_info(
			isp->inputs[isp->input_curr].camera);
		atomisp_set_sensor_mipi_to_isp(mipi_info);

		if ((format->sh_fmt == SH_CSS_FRAME_FORMAT_RAW) &&
		     raw_output_format_match_input(
			mipi_info->input_format, pixelformat))
			return -EINVAL;
	}

	sh_css_input_set_resolution(isp->input_format->out.width,
				    isp->input_format->out.height);

	if (sh_css_input_set_effective_resolution(effective_input_width,
						  effective_input_height))
		return -EINVAL;

	v4l2_info(&atomisp_dev, "effective resolution  %dx%d\n",
		    effective_input_width, effective_input_height);

	if (!isp->vf_format)
		isp->vf_format =
			kzalloc(sizeof(struct atomisp_video_pipe_format),
							GFP_KERNEL);
	if (!isp->vf_format) {
		v4l2_err(&atomisp_dev, "Failed to alloc vf_format memory\n");
		return -ENOMEM;
	}

	isp->vf_format->out.width = 640;
	isp->vf_format->out.height = 480;
	isp->vf_format->out_sh_fmt = SH_CSS_FRAME_FORMAT_YUV420;

	if ((isp->vf_format->out.width > width) ||
	    (isp->vf_format->out.height > height)) {
		if ((width <  640) || (height < 480)) {
			isp->vf_format->out.width = width;
			isp->vf_format->out.height = height;
		} else {
			isp->vf_format->out.width = 640;
			isp->vf_format->out.height = 480;
		}
	}

	switch (isp->sw_contex.run_mode) {
	case CI_MODE_PREVIEW:
		if (sh_css_preview_configure_pp_input(
				effective_input_width, effective_input_height))
			return -EINVAL;

		if (sh_css_preview_configure_output(width, height,
					format->sh_fmt))
			return -EINVAL;

		if (sh_css_preview_get_output_frame_info(output_info))
			return -EINVAL;
		break;
	case CI_MODE_VIDEO:
		if (sh_css_video_configure_viewfinder(
					isp->vf_format->out.width,
					isp->vf_format->out.height,
					isp->vf_format->out_sh_fmt))
			return -EINVAL;

		if (sh_css_video_configure_output(width, height,
						  format->sh_fmt))
			return -EINVAL;

		if (sh_css_video_get_output_frame_info(output_info))
			return -EINVAL;
		break;
	default:
		if (format->sh_fmt == SH_CSS_FRAME_FORMAT_RAW) {
			v4l2_info(&atomisp_dev, "ISP raw mode\n");
			sh_css_capture_set_mode(SH_CSS_CAPTURE_MODE_RAW);
		}

		sh_css_capture_enable_online(isp->params.online_process);

		if (sh_css_capture_configure_pp_input(
						effective_input_width,
						effective_input_height))
			return -EINVAL;

		if (sh_css_capture_configure_viewfinder(
					isp->vf_format->out.width,
					isp->vf_format->out.height,
					isp->vf_format->out_sh_fmt))
			return -EINVAL;

		if (sh_css_capture_configure_output(width, height,
						    format->sh_fmt))
			return -EINVAL;

		ret = sh_css_capture_get_output_frame_info(output_info);
		if (ret) {
			v4l2_err(&atomisp_dev,
				    "Resolution set mismatach error %d\n",
				    ret);
			return -EINVAL;
		}

		if (!isp->params.online_process)
			if (sh_css_capture_get_output_raw_frame_info(
						raw_output_info))
				return -EINVAL;
		if (isp->sw_contex.run_mode != CI_MODE_STILL_CAPTURE) {
			v4l2_err(&atomisp_dev,
				    "Need to set the running mode first\n");
			isp->sw_contex.run_mode = CI_MODE_STILL_CAPTURE;
		}
		break;
	}

	atomisp_update_grid_info(isp);

	/* Free the raw_dump buffer first */
	sh_css_frame_free(isp->raw_output_frame);
	isp->raw_output_frame = NULL;

	if (!isp->params.online_process && !isp->sw_contex.file_input) {
		if (sh_css_frame_allocate_from_info(&isp->raw_output_frame,
						    raw_output_info))
			return -ENOMEM;
	}

	return 0;
}

static int atomisp_get_effective_resolution(struct atomisp_device *isp,
					    unsigned int pixelformat,
					    int out_width, int out_height,
					    int padding_w, int padding_h)
{
	struct atomisp_format_bridge *format;
	struct v4l2_pix_format *in_fmt = &isp->main_format->in;
	struct v4l2_pix_format *out_fmt = &isp->input_format->out;
	unsigned int no_padding_w, no_padding_h;

	format = get_atomisp_format_bridge(pixelformat);
	if (format == NULL)
		return -EINVAL;
	if (!isp->params.yuv_ds_en) {
		in_fmt->width = out_width;
		in_fmt->height = out_height;
		return 0;
	}
	no_padding_w = out_fmt->width - padding_w;
	no_padding_h = out_fmt->height - padding_h;
	/* enable YUV downscaling automatically */
	if (no_padding_w > out_width || no_padding_h > out_height) {
		/* keep a right ratio of width and height*/
		in_fmt->width = no_padding_w;
		in_fmt->height = DIV_RND_UP(in_fmt->width * out_height,
							out_width);
		if (in_fmt->height > no_padding_h) {
			in_fmt->height = no_padding_h;
			in_fmt->width = DIV_RND_UP(in_fmt->height * out_width,
								out_height);

		}
		in_fmt->width = (in_fmt->width &
					~(ATOM_ISP_STEP_WIDTH - 1));
		in_fmt->height = (in_fmt->height &
					~(ATOM_ISP_STEP_HEIGHT - 1));
	} else {
		in_fmt->width = out_width;
		in_fmt->height = out_height;
	}
	return 0;
}

static void atomisp_set_dis_envelop(struct atomisp_device *isp,
			    unsigned int width, unsigned int height,
			    unsigned int *dvs_env_w,
			    unsigned int *dvs_env_h)
{
	/* if subdev type is SOC camera,we do not need to set DVS */
	if (isp->inputs[isp->input_curr].type == SOC_CAMERA)
		isp->params.video_dis_en = 0;

	if (isp->params.video_dis_en &&
	    isp->sw_contex.run_mode == CI_MODE_VIDEO) {
		/* envelope is 20% of the output resolution */
		/*
		 * dvs envelope cannot be round up.
		 * it would cause ISP timeout and color switch issue
		 */
		*dvs_env_w = width / 5;
		*dvs_env_h = height / 5;
		sh_css_video_set_dis_envelope(*dvs_env_w, *dvs_env_h);
	} else {
		/* if DVS gets disabled, make sure it's indeed turned off */
		sh_css_video_set_dis_envelope(0, 0);
	}
}

static int atomisp_get_sensor_bin_factor(struct atomisp_device *isp)
{
	struct v4l2_control ctrl;
	int hbin, vbin;
	int ret;

	memset(&ctrl, 0, sizeof(ctrl));

	ctrl.id = V4L2_CID_BIN_FACTOR_HORZ;
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera, core,
			       g_ctrl, &ctrl);
	hbin = ctrl.value;
	ctrl.id = V4L2_CID_BIN_FACTOR_VERT;
	ret |= v4l2_subdev_call(isp->inputs[isp->input_curr].camera, core,
				g_ctrl, &ctrl);
	vbin = ctrl.value;

	/*
	 * ISP needs to know binning factor from sensor.
	 * In case horizontal and vertical sensor's binning factors
	 * are different or sensor does not support binning factor CID,
	 * ISP will apply default 0 value.
	 */
	if (ret || (hbin != vbin))
		hbin = 0;

	return hbin;
}

static int atomisp_set_fmt_to_snr(struct atomisp_device *isp,
			  struct v4l2_format *f, unsigned int pixelformat,
			  unsigned int padding_w, unsigned int padding_h,
			  unsigned int dvs_env_w, unsigned int dvs_env_h)
{
	struct atomisp_format_bridge *format;
	struct v4l2_mbus_framefmt snr_mbus_fmt;
	struct v4l2_format snr_fmt;
	struct atomisp_video_pipe *out_pipe = &isp->isp_subdev.video_in;
	int ret;

	format = get_atomisp_format_bridge(pixelformat);
	if (format == NULL)
		return -EINVAL;

	if (!isp->sw_contex.file_input) {
		v4l2_fill_mbus_format(&snr_mbus_fmt, &f->fmt.pix,
					format->mbus_code);
		snr_mbus_fmt.height += padding_h + dvs_env_h;
		snr_mbus_fmt.width += padding_w + dvs_env_w;

		ret = v4l2_subdev_call(
				isp->inputs[isp->input_curr].camera,
				video, s_mbus_fmt, &snr_mbus_fmt);
		if (ret)
			return ret;

		isp->input_format->out.width = snr_mbus_fmt.width;
		isp->input_format->out.height = snr_mbus_fmt.height;
		isp->input_format->out.pixelformat =
		    snr_mbus_fmt.code;
		v4l2_info(&atomisp_dev,
			  "sensor output resolution %dx%d\n",
			  isp->input_format->out.width,
			  isp->input_format->out.height);

	} else {	/* file input case */
		isp->input_format->out.width = out_pipe->out_fmt->width;
		isp->input_format->out.height = out_pipe->out_fmt->height;
	}

	if (isp->input_format->out.width < ATOM_ISP_STEP_WIDTH ||
	    isp->input_format->out.height < ATOM_ISP_STEP_HEIGHT)
			return -EINVAL;

	sh_css_input_set_binning_factor(atomisp_get_sensor_bin_factor(isp));

	return 0;
}
void atomisp_get_yuv_ds_status(struct atomisp_device *isp,
			       unsigned int width, unsigned int height)
{
	/* no YUV downscaling if sensor output is 10% larger than isp output */
	unsigned int w_tmp =  isp->input_format->out.width -
				DIV_RND_UP(isp->input_format->out.width, 10);
	unsigned int h_tmp =  isp->input_format->out.height -
				DIV_RND_UP(isp->input_format->out.height , 10);
	/*
	 * yuv downscaling is not enabled in video binary,
	 * ,raw format output, soc sensor. effective resolution should
	 * be the same as isp output.
	 */
	if ((w_tmp < width && h_tmp < height)
		|| isp->sw_contex.run_mode == CI_MODE_VIDEO
		|| isp->sw_contex.bypass
		|| isp->sw_contex.file_input)
		isp->params.yuv_ds_en = false;
	else
		isp->params.yuv_ds_en = true;
	v4l2_info(&atomisp_dev, "yuv downscaling is %s.\n",
	       isp->params.yuv_ds_en ? "enabled" : "disabled");

}
int atomisp_set_fmt(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_format_bridge *format_bridge;
	struct sh_css_frame_info output_info, raw_output_info;
	struct v4l2_format snr_fmt = *f;
	unsigned int width = f->fmt.pix.width;
	unsigned int height = f->fmt.pix.height;
	unsigned int pixelformat = f->fmt.pix.pixelformat;
	unsigned int sh_format;
	unsigned int dvs_env_w = 0,
		     dvs_env_h = 0;
	unsigned int padding_w = pad_w,
		     padding_h = pad_h;
	bool res_overflow = false;
	struct v4l2_streamparm sensor_parm;
	int ret;

	if ((f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
	    (f->type != V4L2_BUF_TYPE_PRIVATE)) {
		v4l2_err(&atomisp_dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	format_bridge = get_atomisp_format_bridge(pixelformat);
	if (format_bridge == NULL)
		return -EINVAL;

	sh_format = format_bridge->sh_fmt;
	if (!pipe->is_main) {
		switch (isp->sw_contex.run_mode) {
		case CI_MODE_VIDEO:
			sh_css_video_configure_viewfinder(
					width, height, sh_format);
			sh_css_video_get_viewfinder_frame_info(&output_info);
			break;
		case CI_MODE_STILL_CAPTURE:
			sh_css_capture_configure_viewfinder(
					width, height, sh_format);
			sh_css_capture_get_viewfinder_frame_info(&output_info);
			break;
		}
		goto done;
	}

	/* V4L2_BUF_TYPE_PRIVATE will set offline processing */
	if (f->type == V4L2_BUF_TYPE_PRIVATE)
		isp->params.online_process = 0;
	else
		isp->params.online_process = 1;

	/* setting run mode to the sensor */
	sensor_parm.parm.capture.capturemode = isp->sw_contex.run_mode;
	v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				video, s_parm, &sensor_parm);

	/* get sensor resolution and format */
	snr_fmt = *f;
	atomisp_try_fmt(vdev, &snr_fmt, &res_overflow);
	width = snr_fmt.fmt.pix.width;
	height = snr_fmt.fmt.pix.height;

	/*
	 * bypass mode is enabled when sensor output format is not raw
	 * or isp output format is raw.
	 */
	if (isp->inputs[isp->input_curr].type != TEST_PATTERN &&
		(!is_pixelformat_raw(snr_fmt.fmt.pix.pixelformat) ||
			is_pixelformat_raw(pixelformat))) {
		isp->sw_contex.bypass = true;
		padding_h = 0;
		padding_w = 0;
	} else
		isp->sw_contex.bypass = false;

	/* construct resolution supported by isp */
	if (res_overflow) {
		width -= padding_w;
		height -= padding_h;
		/* app vs isp */
		width = min(width, (u32)ATOM_ISP_MAX_WIDTH);
		height = min(height, (u32)ATOM_ISP_MAX_HEIGHT);

		width = max(width, (u32)ATOM_ISP_MIN_WIDTH);
		height = max(height, (u32)ATOM_ISP_MIN_HEIGHT);

		width = width - width % ATOM_ISP_STEP_WIDTH;
		height = height - height % ATOM_ISP_STEP_HEIGHT;

		f->fmt.pix.width = width;
		f->fmt.pix.height = height;
	}

	/* set dis envelop if video and dis are enabled */
	atomisp_set_dis_envelop(isp, width, height, &dvs_env_w, &dvs_env_h);

	if (!isp->input_format)
		isp->input_format =
		kzalloc(sizeof(struct atomisp_video_pipe_format), GFP_KERNEL);
	if (!isp->input_format) {
		v4l2_err(&atomisp_dev, "Failed to alloc input_format memory\n");
		return -ENOMEM;
	}

	/* set format info to sensor */
	ret = atomisp_set_fmt_to_snr(isp, f, pixelformat,
				     padding_w, padding_h,
				     dvs_env_w, dvs_env_h);
	if (ret)
		return -EINVAL;

	/* Only main stream pipe will be here */
	isp->main_format = pipe->format;
	if (!isp->main_format) {
		v4l2_err(&atomisp_dev, "Internal error\n");
		return -EINVAL;
	}

	atomisp_get_yuv_ds_status(isp, width, height);

	/*
	 * calculate effective solution to enable yuv downscaling and keep
	 * ratio of width and height
	 */
	ret = atomisp_get_effective_resolution(isp, pixelformat, width, height,
					       padding_w, padding_h);
	if (ret)
		return -EINVAL;

	/* set format to isp */
	ret = atomisp_set_fmt_to_isp(vdev, &output_info, &raw_output_info,
				     width, height, pixelformat);
	if (ret)
		return -EINVAL;
done:
	pipe->format->out.width = width;
	pipe->format->out.height = height;
	pipe->format->out.pixelformat = pixelformat;
	pipe->format->out.bytesperline =
	    DIV_RND_UP(format_bridge->depth * output_info.padded_width, 8);
	pipe->format->out.sizeimage =
	    PAGE_ALIGN(height * pipe->format->out. bytesperline);
	if (f->fmt.pix.field == V4L2_FIELD_ANY)
		f->fmt.pix.field = V4L2_FIELD_NONE;
	pipe->format->out.field = f->fmt.pix.field;
	pipe->format->out_sh_fmt = sh_format;

	memcpy(&f->fmt.pix, &pipe->format->out,
			sizeof(struct v4l2_pix_format));
	f->fmt.pix.priv = PAGE_ALIGN(pipe->format->out.width *
				     pipe->format->out.height * 2);

	pipe->capq.field = f->fmt.pix.field;

	v4l2_info(&atomisp_dev,
		    "ISP output resolution: %dx%d, format = %s\n",
		    pipe->format->out.width, pipe->format->out.height,
		    format_bridge->description);
	return 0;
}

int atomisp_set_fmt_file(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	enum sh_css_input_format sh_input_format;
	int ret = 0;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
				"Wrong v4l2 buf type for output\n");
		return -EINVAL;
	}

	switch (isp->sw_contex.output_mode) {
	case OUTPUT_MODE_FILE:
		ret = atomisp_try_fmt_file(isp, f);
		if (ret)
			return ret;

		pipe->out_fmt->pixelformat = f->fmt.pix.pixelformat;
		pipe->out_fmt->framesize = f->fmt.pix.sizeimage;
		pipe->out_fmt->imagesize = f->fmt.pix.sizeimage;
		pipe->out_fmt->depth = get_pixel_depth(f->fmt.pix.pixelformat);
		pipe->out_fmt->bytesperline = f->fmt.pix.bytesperline;

		pipe->out_fmt->bayer_order = f->fmt.pix.priv;
		pipe->out_fmt->width = f->fmt.pix.width;
		pipe->out_fmt->height = f->fmt.pix.height;
		sh_input_format = get_sh_input_format(
						pipe->out_fmt->pixelformat);
		if (sh_input_format == -EINVAL) {
			v4l2_err(&atomisp_dev,
					"Wrong v4l2 format for output\n");
			return -EINVAL;
		}

		sh_css_input_set_format(sh_input_format);
		sh_css_input_set_mode(SH_CSS_INPUT_MODE_FIFO);
		sh_css_input_set_bayer_order(pipe->out_fmt->bayer_order);
		sh_css_input_configure_port(SH_CSS_MIPI_PORT_4LANE, 2, 0xffff4);
		return 0;

	case OUTPUT_MODE_TEXT:
		pipe->out_fmt->framesize = f->fmt.pix.sizeimage;
		pipe->out_fmt->imagesize = f->fmt.pix.sizeimage;
		return 0;

	default:
		v4l2_err(&atomisp_dev, "Unspported output mode\n");
		return -EINVAL;
	}
}

void atomisp_free_all_shading_tables(struct atomisp_device *isp)
{
	int i;

	for (i = 0; i < isp->input_cnt; i++) {
		if (isp->inputs[i].shading_table == NULL)
			continue;
		sh_css_shading_table_free(isp->inputs[i].shading_table);
		isp->inputs[i].shading_table = NULL;
	}
}

int atomisp_set_shading_table(struct atomisp_device *isp,
		struct atomisp_shading_table *user_shading_table)
{
	struct sh_css_shading_table *shading_table;
	struct sh_css_shading_table *free_table;
	unsigned int len_table;
	int i;
	int ret = 0;

	if (!user_shading_table)
		return -EINVAL;

	if (user_shading_table->flags & ATOMISP_SC_FLAG_QUERY) {
		user_shading_table->enable = isp->params.sc_en;
		return 0;
	}

	if (!user_shading_table->enable) {
		mutex_lock(&isp->isp_lock);
		sh_css_set_shading_table(NULL);
		isp->params.sc_en = 0;
		mutex_unlock(&isp->isp_lock);
		return 0;
	}

	/* If enabling, all tables must be set */
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		if (!user_shading_table->data[i])
			return -EINVAL;
	}

	/* Shading table size per color */
	if (user_shading_table->width > SH_CSS_MAX_SCTBL_WIDTH_PER_COLOR ||
	    user_shading_table->height > SH_CSS_MAX_SCTBL_HEIGHT_PER_COLOR)
		return -EINVAL;

	shading_table = sh_css_shading_table_alloc(user_shading_table->width,
						   user_shading_table->height);
	if (!shading_table)
		return -ENOMEM;

	len_table = user_shading_table->width * user_shading_table->height *
		    ATOMISP_SC_TYPE_SIZE;
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		ret = copy_from_user(shading_table->data[i],
				     user_shading_table->data[i], len_table);
		if (ret) {
			free_table = shading_table;
			ret = -EFAULT;
			goto out;
		}
	}
	shading_table->sensor_width = user_shading_table->sensor_width;
	shading_table->sensor_height = user_shading_table->sensor_height;
	shading_table->fraction_bits = user_shading_table->fraction_bits;

	mutex_lock(&isp->isp_lock);

	free_table = isp->inputs[isp->input_curr].shading_table;
	isp->inputs[isp->input_curr].shading_table = shading_table;
	sh_css_set_shading_table(shading_table);
	isp->params.sc_en = 1;

	mutex_unlock(&isp->isp_lock);

out:
	if (free_table != NULL)
		sh_css_shading_table_free(free_table);

	return ret;
}

/*
 * atomisp_acc_get_fw - Search for firmware with given handle
 *
 * This function will search for given handle until:
 * - Handle is found
 * - Function has found total number of non-NULL slots
 *
 * It's ensured by atomisp_acc_fw_alloc() number of used slots is never bigger
 * then ATOMISP_ACC_FW_MAX.
 */
static struct sh_css_acc_fw *
atomisp_acc_get_fw(struct atomisp_device *isp, unsigned int handle)
{
	int i = -1;
	int count = isp->acc_fw_count;

	while (count) {
		i++;
		if (unlikely(i == ATOMISP_ACC_FW_MAX)) {
			/* Sanity check. If code reaches here, we've a bug. */
			WARN_ON(1);
			return NULL;
		}
		if (isp->acc_fw[i] == NULL)
			continue;
		if (isp->acc_fw[i]->header.handle == handle)
			break;
		count--;
	}

	return count ? isp->acc_fw[i] : NULL;
}

/*
 * atomisp_acc_get_index - Search for firmware index in isp->acc_fw[] array
 *
 * This function will search for firmware index until:
 * - Given firmware is found
 * - Function has searched all non-NULL slots
 *
 * It's ensured by atomisp_acc_fw_alloc() number of used slots is never bigger
 * then ATOMISP_ACC_FW_MAX.
 */
static int
atomisp_acc_get_index(struct atomisp_device *isp, struct sh_css_acc_fw *fw)
{
	int i = -1;
	int count = isp->acc_fw_count;

	while (count) {
		i++;
		if (unlikely(i == ATOMISP_ACC_FW_MAX)) {
			/* Sanity check. If code reaches here, we've a bug. */
			WARN_ON(1);
			return -EINVAL;
		}
		if (isp->acc_fw[i] == NULL)
			continue;
		if (isp->acc_fw[i] == fw)
			break;
		count--;
	}

	return count ? i : -EINVAL;
}

static void
atomisp_acc_fw_free_args(struct atomisp_device *isp, struct sh_css_acc_fw *fw)
{
	int i = atomisp_acc_get_index(isp, fw);

	/* Sanity check */
	if (i < 0) {
		WARN_ON(1);
		return;
	}

	/* Free `host' data for all arguments */
	for (i = 0; ; i++) {
		union host *host;
		enum sh_css_acc_arg_type type;

		host = sh_css_argument_get_host(fw, i);
		if (sh_css_argument_set_host(fw, i, NULL))
			break;		/* Failure: all arguments freed */
		if (!host)
			continue;
		type = sh_css_argument_type(fw, i);
		switch (type) {
		case SH_CSS_ACC_ARG_SCALAR_IN:
			kfree(host->scalar.kernel_ptr);
			break;
		case ATOMISP_ACC_ARG_FRAME:
		case SH_CSS_ACC_ARG_PTR_IN:
		case SH_CSS_ACC_ARG_PTR_OUT:
		case SH_CSS_ACC_ARG_PTR_IO:
			hrt_isp_css_mm_free(host->ptr.hmm_ptr);
			break;
		default:
			v4l2_err(&atomisp_dev, "%s: Invalid argument type.\n",
				 __func__);
			break;
		}
		kfree(host);
	}
}

static void
atomisp_acc_fw_free(struct atomisp_device *isp, struct sh_css_acc_fw *fw)
{
	int i = atomisp_acc_get_index(isp, fw);

	/* Sanity check */
	if (i < 0) {
		WARN_ON(1);
		return;
	}

	isp->acc_fw[i] = NULL;
	isp->acc_fw_count--;

	kfree(fw);
}

static struct sh_css_acc_fw *
atomisp_acc_fw_alloc(struct atomisp_device *isp,
		     struct atomisp_acc_fw_load *user_fw)
{
	struct sh_css_acc_fw *fw;
	int ret;
	int i;

	/* REVISIT: does size need to be multiple of page size? */
	fw = kzalloc(user_fw->size, GFP_KERNEL);

	if (fw == NULL) {
		v4l2_err(&atomisp_dev, "%s: Failed to alloc acc fw blob\n",
			 __func__);
		ret = -ENOMEM;
		goto err;
	}

	ret = copy_from_user(fw, user_fw->data, user_fw->size);
	if (ret) {
		v4l2_err(&atomisp_dev, "%s: Failed to copy acc fw blob\n",
			 __func__);
		ret = -EIO;
		goto err;
	}

	mutex_lock(&isp->isp_lock);
	if (isp->acc_fw_count >= ATOMISP_ACC_FW_MAX) {
		mutex_unlock(&isp->isp_lock);
		ret = -EINVAL;
		goto err;
	}

	/* Find first free slot */
	for (i = 0; i < ATOMISP_ACC_FW_MAX; i++) {
		if (isp->acc_fw[i] == NULL)
			break;
	}
	user_fw->fw_handle = isp->acc_fw_handle;
	fw->header.handle = isp->acc_fw_handle;
	isp->acc_fw[i] = fw;

	isp->acc_fw_handle++;
	isp->acc_fw_count++;
	mutex_unlock(&isp->isp_lock);

	return fw;

err:
	kfree(fw);
	return ERR_PTR(ret);
}

int atomisp_acc_load(struct atomisp_device *isp,
		     struct atomisp_acc_fw_load *user_fw)
{
	struct sh_css_acc_fw *fw;
	int ret;

	mutex_lock(&isp->input_lock);
	fw = atomisp_acc_fw_alloc(isp, user_fw);
	if (IS_ERR(fw)) {
		v4l2_err(&atomisp_dev, "%s: Acceleration firmware allocation "
				       "failed\n", __func__);
		ret = PTR_ERR(fw);
		goto out;
	}

	mutex_lock(&isp->isp_lock);
	ret = sh_css_load_acceleration(fw);
	if (ret) {
		atomisp_acc_fw_free(isp, fw);
		mutex_unlock(&isp->isp_lock);
		v4l2_err(&atomisp_dev, "%s: Failed to load acceleration "
				       "firmware\n", __func__);
		ret = -EAGAIN;
		goto out;
	}
	mutex_unlock(&isp->isp_lock);
	init_completion(&isp->acc_fw_complete);

out:
	mutex_unlock(&isp->input_lock);
	return ret;
}

int atomisp_acc_unload(struct atomisp_device *isp, unsigned int *handle)
{
	struct sh_css_acc_fw *fw;
	int ret = 0;

	mutex_lock(&isp->input_lock);
	fw = atomisp_acc_get_fw(isp, *handle);

	if (fw == NULL) {
		v4l2_err(&atomisp_dev, "%s: Invalid acceleration firmware "
				       "handle\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (!fw->header.loaded) {
		ret = -EINVAL;
		goto out;
	}

	if (isp->marked_fw_for_unload != NULL) {
		v4l2_err(&atomisp_dev, "%s: Acceleration firmware unload "
				       "pending\n", __func__);
		ret = -EBUSY;
		goto out;
	}

	if (isp->sw_contex.isp_streaming == false) {
		/* We're not streaming, so it's safe to unload now */
		mutex_lock(&isp->isp_lock);
		atomisp_acc_fw_free_args(isp, fw);
		sh_css_unload_acceleration(fw);
		atomisp_acc_fw_free(isp, fw);
		mutex_unlock(&isp->isp_lock);
		ret = 0;
		goto out;
	}

	/*
	 * If we're streaming, unload should be synced with end of frame.
	 * We mark it for unload on atomisp_work() and wait for it.
	 */
	init_completion(&isp->acc_unload_fw_complete);
	isp->marked_fw_for_unload = fw;
	mutex_unlock(&isp->input_lock);
	wait_for_completion_timeout(&isp->acc_unload_fw_complete, 1 * HZ);

	return 0;

out:
	mutex_unlock(&isp->input_lock);
	return ret;
}

int atomisp_acc_set_arg(struct atomisp_device *isp,
			struct atomisp_acc_fw_arg *fw_arg)
{
	struct sh_css_acc_fw *fw;
	enum atomisp_acc_arg_type type;
	void *frame_ptr;
	unsigned int handle = fw_arg->fw_handle;
	unsigned int index = fw_arg->index;
	unsigned int size = fw_arg->size;
	unsigned int pgnr;
	union host *host;
	int ret;

	mutex_lock(&isp->input_lock);
	fw = atomisp_acc_get_fw(isp, handle);
	if (fw == NULL) {
		v4l2_err(&atomisp_dev, "Invalid fw handle\n");
		ret = -EINVAL;
		goto out;
	}

	ret = sh_css_set_acceleration_argument(fw, index, NULL, 0);
	if (ret != 0)
		goto out;

	/* Allocate `host' union if not already done so */
	host = (union host *)sh_css_argument_get_host(fw, index);

	if (!host) {
		/* Allocate new */
		host = kzalloc(sizeof(*host), GFP_KERNEL);
		if (!host) {
			ret = -ENOMEM;
			goto out;
		}
		ret = sh_css_argument_set_host(fw, index, host);
		if (ret != 0) {
			kfree(host);
			goto out;
		}
	}

	type = sh_css_argument_type(fw, index);
	switch (type) {
	case SH_CSS_ACC_ARG_SCALAR_IN:
		/* Free old argument data if one already exists */#
			kfree(host->scalar.kernel_ptr);

		/* Allocate and copy data into kernel space */
		host->scalar.kernel_ptr = kmalloc(size, GFP_KERNEL);
		if (!host->scalar.kernel_ptr)
			return -ENOMEM;
		if (copy_from_user(host->scalar.kernel_ptr,
					fw_arg->value, size)) {
			kfree(host->scalar.kernel_ptr);
			host->scalar.kernel_ptr = NULL;
			return -EFAULT;
		}
		host->scalar.size = size;
		host->scalar.user_ptr = fw_arg->value;
		ret = sh_css_set_acceleration_argument(fw, index,
					host->scalar.kernel_ptr, size);
		break;
	case ATOMISP_ACC_ARG_FRAME:
	case SH_CSS_ACC_ARG_PTR_IN:
	case SH_CSS_ACC_ARG_PTR_OUT:
	case SH_CSS_ACC_ARG_PTR_IO:
		/* Free old argument data if one already exists */
		hrt_isp_css_mm_free(host->ptr.hmm_ptr);
		pgnr = (size + (PAGE_SIZE - 1)) >> PAGE_SHIFT;

		mutex_lock(&isp->isp_lock);
		frame_ptr = hrt_isp_css_mm_alloc_user_ptr(size,
					(unsigned int)fw_arg->value, pgnr);
		mutex_unlock(&isp->isp_lock);

		if (IS_ERR_OR_NULL(frame_ptr)) {
			v4l2_err(&atomisp_dev, "%s: Failed to allocate frame "
				 "for acceleration firmware\n", __func__);
			ret = -EINVAL;
			goto out;
		}

		host->ptr.hmm_ptr = frame_ptr;
		ret = sh_css_set_acceleration_argument(fw, index, frame_ptr,
						       size);
		break;


	default:
		v4l2_err(&atomisp_dev, "Invalid fw argument type\n");
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&isp->input_lock);
	return ret;
}

int atomisp_acc_start(struct atomisp_device *isp, unsigned int *handle)
{
	struct sh_css_acc_fw *fw;
	unsigned int ret;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);
	fw = atomisp_acc_get_fw(isp, *handle);
	if (fw == NULL) {
		v4l2_err(&atomisp_dev, "%s: Invalid firmware handle\n",
			 __func__);
		mutex_unlock(&isp->isp_lock);
		ret = -EINVAL;
		goto out;
	}

	ret = sh_css_start_acceleration(fw);
	mutex_unlock(&isp->isp_lock);
	if (ret) {
		v4l2_err(&atomisp_dev, "%s: Failed to start acceleration "
				       "firmware\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	/* Initialize the interrupt here if it's a standalone binary */
	if (fw->header.type == ATOMISP_ACC_STANDALONE)
		INIT_COMPLETION(isp->acc_fw_complete);

out:
	mutex_unlock(&isp->input_lock);
	return ret;
}

int atomisp_acc_wait(struct atomisp_device *isp, unsigned int *handle)
{
	struct sh_css_acc_fw *fw;
	unsigned int time_left;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	fw = atomisp_acc_get_fw(isp, *handle);

	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);

	if (fw == NULL) {
		v4l2_err(&atomisp_dev, "Invalid fw handle\n");
		return -EINVAL;
	}

	if (fw->header.type != ATOMISP_ACC_STANDALONE)
		return -EINVAL;

	time_left =
	    wait_for_completion_timeout(&isp->acc_fw_complete,
					1 * HZ);

	/* Timeout to get the acc fw */
	if (time_left == 0) {
		v4l2_err(&atomisp_dev,
			 "%s: Acceleration firmware timeout to finish\n",
			 __func__);
		return -EINVAL;
	}

	mutex_lock(&isp->isp_lock);
	sh_css_acceleration_done(fw);
	mutex_unlock(&isp->isp_lock);

	return 0;
}

int atomisp_acc_abort(struct atomisp_device *isp,
		      struct atomisp_acc_fw_abort *abort)
{
	struct sh_css_acc_fw *fw;
	int ret = 0;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	fw = atomisp_acc_get_fw(isp, abort->fw_handle);
	if (fw == NULL) {
		v4l2_err(&atomisp_dev, "%s: Invalid firmware handle\n",
			 __func__);
		ret = -EINVAL;
		goto out;
	}

	sh_css_abort_acceleration(fw, abort->timeout);

out:
	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);

	return ret;
}

int atomisp_save_iunit_reg(struct atomisp_device *isp)
{
	/*Clear those register value first*/
	isp->hw_contex.pcicmdsts = 0;
	isp->hw_contex.ispmmadr = 0;
	isp->hw_contex.msicap = 0;
	isp->hw_contex.msi_addr = 0;
	isp->hw_contex.msi_data = 0;
	isp->hw_contex.intr = 0;
	isp->hw_contex.interrupt_control = 0;
	isp->hw_contex.pmcs = 0;
	isp->hw_contex.cg_dis = 0;
	isp->hw_contex.i_control = 0;
	isp->hw_contex.pci_cmd = 0;
	isp->hw_contex.csi_rcomp_config = 0;
	isp->hw_contex.csi_afe_dly = 0;
	isp->hw_contex.csi_control = 0;

	isp->hw_contex.pcicmdsts = atomisp_msg_read32(isp, IUNIT_PORT,
						      PCICMDSTS);
	isp->hw_contex.ispmmadr = atomisp_msg_read32(isp, IUNIT_PORT,
						     ISPMMADR);
	isp->hw_contex.msicap = atomisp_msg_read32(isp, IUNIT_PORT,
						   MSI_CAPID);
	isp->hw_contex.msi_addr = atomisp_msg_read32(isp, IUNIT_PORT,
						     MSI_ADDRESS);
	isp->hw_contex.msi_data = atomisp_msg_read32(isp, IUNIT_PORT,
						     MSI_DATA);
	isp->hw_contex.intr = atomisp_msg_read32(isp, IUNIT_PORT, INTR);
	isp->hw_contex.interrupt_control = atomisp_msg_read32(isp, IUNIT_PORT,
							      INTR_CTL);
	isp->hw_contex.pmcs = atomisp_msg_read32(isp, IUNIT_PORT, PMCS);
	isp->hw_contex.cg_dis = atomisp_msg_read32(isp, IUNIT_PORT, CG_DIS);
	isp->hw_contex.i_control = atomisp_msg_read32(isp, IUNIT_PORT,
						      I_CONTROL);
	isp->hw_contex.csi_rcomp_config = atomisp_msg_read32(isp, IUNITPHY_PORT,
							     CSI_RCOMP);
	isp->hw_contex.csi_afe_dly = atomisp_msg_read32(isp, IUNITPHY_PORT,
							CSI_AFE);
	isp->hw_contex.csi_control = atomisp_msg_read32(isp, IUNITPHY_PORT,
							CSI_CONTROL);

	pci_read_config_word(isp->pdev, PCI_COMMAND, &isp->hw_contex.pci_cmd);


	return 0;
}

int atomisp_restore_iunit_reg(struct atomisp_device *isp)
{
	pci_write_config_word(isp->pdev, PCI_COMMAND, isp->hw_contex.pci_cmd);

	atomisp_msg_write32(isp, IUNIT_PORT, PCICMDSTS,
			    isp->hw_contex.pcicmdsts);
	atomisp_msg_write32(isp, IUNIT_PORT, ISPMMADR,
			    isp->hw_contex.ispmmadr);
	atomisp_msg_write32(isp, IUNIT_PORT, MSI_CAPID,
			    isp->hw_contex.msicap);
	atomisp_msg_write32(isp, IUNIT_PORT, MSI_ADDRESS,
			    isp->hw_contex.msi_addr);
	atomisp_msg_write32(isp, IUNIT_PORT, MSI_DATA,
			    isp->hw_contex.msi_data);
	atomisp_msg_write32(isp, IUNITPHY_PORT, CSI_CONTROL, 0xFF0003);
	atomisp_msg_write32(isp, IUNIT_PORT, INTR, isp->hw_contex.intr);
	atomisp_msg_write32(isp, IUNIT_PORT, INTR_CTL,
				isp->hw_contex.interrupt_control);
	atomisp_msg_write32(isp, IUNIT_PORT, PMCS, isp->hw_contex.pmcs);
	atomisp_msg_write32(isp, IUNIT_PORT, CG_DIS, isp->hw_contex.cg_dis);
	atomisp_msg_write32(isp, IUNIT_PORT, I_CONTROL,
			    isp->hw_contex.i_control);
	atomisp_msg_write32(isp, IUNITPHY_PORT, CSI_RCOMP,
				isp->hw_contex.csi_rcomp_config);
	atomisp_msg_write32(isp, IUNITPHY_PORT, CSI_AFE,
			    isp->hw_contex.csi_afe_dly);
	atomisp_msg_write32(isp, IUNITPHY_PORT, CSI_CONTROL,
				isp->hw_contex.csi_control);

	return 0;
}

/*Turn off ISP power island*/
int atomisp_ospm_power_island_down(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;
	int timeout = 100;
	bool idle;

	/* if ISP timeout, we can force powerdown */
	if (isp->isp_timeout) {
		isp->isp_timeout = false;
		goto done;
	}

	idle = sh_css_hrt_system_is_idle();
	while (!idle && timeout--) {
		udelay(20);
		idle = sh_css_hrt_system_is_idle();
	}

	if (timeout < 0) {
		v4l2_err(&atomisp_dev,
			 "Timeout to stop ISP HW\n");
		return -EINVAL;
	}

done:
	/* power down DPHY */
	pwr_cnt = atomisp_msg_read32(isp, IUNITPHY_PORT, CSI_CONTROL);
	pwr_cnt |= 0x300;
	atomisp_msg_write32(isp, IUNITPHY_PORT, CSI_CONTROL, pwr_cnt);

	/* Power down IPH/ISP */
	if (pmu_nc_set_power_state(APM_ISP_ISLAND | APM_IPH_ISLAND,
			       OSPM_ISLAND_DOWN, APM_REG_TYPE))
		return -EINVAL;

	isp->sw_contex.power_state = ATOM_ISP_POWER_DOWN;
	return 0;
}

/*Turn on ISP power island*/
int atomisp_ospm_power_island_up(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;

	/* Power up IPH/ISP */
	if (pmu_nc_set_power_state(APM_ISP_ISLAND | APM_IPH_ISLAND,
			       OSPM_ISLAND_UP, APM_REG_TYPE))
		return -EINVAL;

	/* power on DPHY */
	pwr_cnt = atomisp_msg_read32(isp, IUNITPHY_PORT, CSI_CONTROL);
	pwr_cnt &= ~0x300;
	atomisp_msg_write32(isp, IUNITPHY_PORT, CSI_CONTROL, pwr_cnt);

	isp->sw_contex.power_state = ATOM_ISP_POWER_UP;
	return 0;
}


int atomisp_exif_makernote(struct atomisp_device *isp,
	void *config)
{
	struct atomisp_makernote_info *arg =
			(struct atomisp_makernote_info *)config;
	struct v4l2_control ctrl;

	ctrl.id = V4L2_CID_FOCAL_ABSOLUTE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				 core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev, "failed to g_ctrl for focal length\n");
	else
		arg->focal_length = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_ABSOLUTE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev, "failed to g_ctrl for f-number\n");
	else
		arg->f_number_curr = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_RANGE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev,
				"failed to g_ctrl for f number range\n");
	else
		arg->f_number_range = ctrl.value;

	return 0;
}

int atomisp_flash_enable(struct atomisp_device *isp, int num_frames)
{
	if (num_frames <= 0)
		return -EINVAL;
	/* a requested flash is still in progress. */
	if (isp->params.num_flash_frames)
		return -EBUSY;

	isp->params.num_flash_frames = num_frames;
	return 0;
}
