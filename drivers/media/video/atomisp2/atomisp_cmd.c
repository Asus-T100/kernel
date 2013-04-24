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
#include <linux/firmware.h>
#include <linux/intel_mid_pm.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/pm_runtime.h>
#include <linux/timer.h>

#include <asm/intel-mid.h>

#include <media/v4l2-event.h>
#include <media/videobuf-vmalloc.h>

#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_fops.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"
#include "atomisp-regs.h"
#include "atomisp_tables.h"
#include "atomisp_acc.h"
#include "atomisp_compat.h"

#include "hrt/hive_isp_css_mm_hrt.h"

#include "sh_css_debug.h"
#include "sh_css_hrt.h"
#include "sh_css_types.h"
#include "sh_css_accelerate.h"
#include "sh_css_defs.h"
#include "sh_css.h"
#include "system_global.h"
#include "sh_css_internal.h"
#include "sh_css_sp.h"
#include "gp_device.h"
#include "device_access.h"
#include "irq.h"

#include "hrt/bits.h"

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

/*
 * atomisp_kernel_malloc: chooses whether kmalloc() or vmalloc() is preferable.
 *
 * It is also a wrap functions to pass into css framework.
 */
void *atomisp_kernel_malloc(size_t bytes)
{
	/* vmalloc() is preferable if allocating more than 1 page */
	if (bytes > PAGE_SIZE)
		return vmalloc(bytes);

	return kmalloc(bytes, GFP_KERNEL);
}

/*
 * atomisp_kernel_zalloc: chooses whether set 0 to the allocated memory.
 *
 * It is also a wrap functions to pass into css framework.
 */
void *atomisp_kernel_zalloc(size_t bytes, bool zero_mem)
{
	void *ptr = atomisp_kernel_malloc(bytes);

	if (ptr && zero_mem)
		memset(ptr, 0, bytes);

	return ptr;
}

/*
 * Free buffer allocated with atomisp_kernel_malloc()/atomisp_kernel_zalloc
 * helper
 */
void atomisp_kernel_free(void *ptr)
{
	/* Verify if buffer was allocated by vmalloc() or kmalloc() */
	if (is_vmalloc_addr(ptr))
		vfree(ptr);
	else
		kfree(ptr);
}

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

/* This is just a draft rules, should be tuned when sensor is ready*/
static struct atomisp_freq_scaling_rule dfs_rules[] = {
	{
		.width = ISP_FREQ_RULE_ANY,
		.height = ISP_FREQ_RULE_ANY,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_VIDEO,
	},
	{
		.width = 1920,
		.height = 1080,
		.fps = 60,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_VIDEO,
	},
	{
		.width = 4192,
		.height = 3104,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = 4096,
		.height = 3072,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = 3648,
		.height = 2736,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = ISP_FREQ_RULE_ANY,
		.height = ISP_FREQ_RULE_ANY,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = 1280,
		.height = 720,
		.fps = 60,
		.isp_freq = ISP_FREQ_320MHZ,
		.run_mode = ATOMISP_RUN_MODE_VIDEO,
	},
};

#define ISP_FREQ_RULE_MAX (ARRAY_SIZE(dfs_rules))

static unsigned short atomisp_get_sensor_fps(struct atomisp_device *isp)
{
	struct v4l2_subdev_frame_interval frame_interval;
	unsigned short fps;

	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
		video, g_frame_interval, &frame_interval)) {
		fps = 0;
	} else {
		if (frame_interval.interval.numerator)
			fps = frame_interval.interval.denominator /
			    frame_interval.interval.numerator;
		else
			fps = 0;
	}
	return fps;
}
/*
 * DFS progress is shown as follows:
 * 1. Target frequency is calculated according to FPS/Resolution/ISP running
 * mode.
 * 2. Ratio is calucated in formula: 2 * (HPLL / target frequency) - 1
 * 3. Set ratio to ISPFREQ40, 1 to FREQVALID and ISPFREQGUAR40
 *    to 200MHz in ISPSSPM1.
 * 4. Wait for FREQVALID to be cleared by P-Unit.
 * 5. Wait for field ISPFREQSTAT40 in ISPSSPM1 turn to ratio set in 3.
 */
static int write_target_freq_to_hw(struct atomisp_device *isp, int new_freq)
{
	int ratio, timeout;
	u32 isp_sspm1 = 0;

	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	if (isp_sspm1 & ISP_FREQ_VALID_MASK) {
		dev_dbg(isp->dev, "clearing ISPSSPM1 valid bit.\n");
		intel_mid_msgbus_write32(PUNIT_PORT, ISPSSPM1,
				    isp_sspm1 & ~(1 << ISP_FREQ_VALID_OFFSET));
	}

	ratio = 2 * (HPLL_FREQ / new_freq) - 1;
	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	isp_sspm1 &= ~(0x1F << ISP_REQ_FREQ_OFFSET);
	intel_mid_msgbus_write32(PUNIT_PORT, ISPSSPM1,
				   isp_sspm1
				   | ratio << ISP_REQ_FREQ_OFFSET
				   | 1 << ISP_FREQ_VALID_OFFSET
				   | 0xF << ISP_REQ_GUAR_FREQ_OFFSET);

	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	timeout = 10;
	while ((isp_sspm1 & ISP_FREQ_VALID_MASK) && timeout) {
		isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
		dev_dbg(isp->dev, "waiting for ISPSSPM1 valid bit to be 0.\n");
		udelay(100);
		timeout--;
	}
	if (timeout == 0) {
		dev_err(isp->dev, "DFS failed due to HW error.\n");
		return -EINVAL;
	}

	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	timeout = 10;
	while (((isp_sspm1 >> ISP_FREQ_STAT_OFFSET) != ratio) && timeout) {
		isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
		dev_dbg(isp->dev, "waiting for ISPSSPM1 status bit to be "
				"0x%x.\n", new_freq);
		udelay(100);
		timeout--;
	}
	if (timeout == 0) {
		dev_err(isp->dev, "DFS target freq is rejected by HW.\n");
		return -EINVAL;
	}

	return 0;
}
int atomisp_freq_scaling(struct atomisp_device *isp, enum atomisp_dfs_mode mode)
{
	struct atomisp_sub_device *asd = &isp->isp_subdev;
	unsigned int new_freq;
	struct atomisp_freq_scaling_rule curr_rules;
	int i, ret;
	unsigned short fps = 0;

	if (isp->sw_contex.power_state != ATOM_ISP_POWER_UP) {
		dev_err(isp->dev, "DFS cannot proceed due to no power.\n");
		return -EINVAL;
	}

	if (mode == ATOMISP_DFS_MODE_LOW) {
		new_freq = ISP_FREQ_200MHZ;
		goto done;
	}

	if (mode == ATOMISP_DFS_MODE_MAX) {
		new_freq = ISP_FREQ_400MHZ;
		goto done;
	}

	fps = atomisp_get_sensor_fps(isp);
	if (fps == 0)
		return -EINVAL;

	curr_rules.width = asd->fmt[asd->capture_pad].fmt.width;
	curr_rules.height = asd->fmt[asd->capture_pad].fmt.height;
	curr_rules.fps = fps;
	curr_rules.run_mode = isp->isp_subdev.run_mode->val;
	/*
	 * For continuous mode, we need to make the capture setting applied
	 * since preview mode, because there is no chance to do this when
	 * starting image capture.
	 */
	if (isp->isp_subdev.continuous_mode->val)
		curr_rules.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE;

	/* search for the target frequency by looping freq rules*/
	for (i = 0; i < ISP_FREQ_RULE_MAX; i++) {
		if (curr_rules.width != dfs_rules[i].width
			&& dfs_rules[i].width != ISP_FREQ_RULE_ANY)
			continue;
		if (curr_rules.height != dfs_rules[i].height
			&& dfs_rules[i].height != ISP_FREQ_RULE_ANY)
			continue;
		if (curr_rules.fps != dfs_rules[i].fps
			&& dfs_rules[i].fps != ISP_FREQ_RULE_ANY)
			continue;
		if (curr_rules.run_mode != dfs_rules[i].run_mode
			&& dfs_rules[i].run_mode != ISP_FREQ_RULE_ANY)
			continue;
		break;
	}
	if (i == ISP_FREQ_RULE_MAX)
		new_freq = ISP_FREQ_320MHZ;
	else
		new_freq = dfs_rules[i].isp_freq;

done:
	/* workround to get isp works at 400Mhz for byt due to perf issue */
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		new_freq = ISP_FREQ_400MHZ;

	dev_dbg(isp->dev, "DFS target frequency=%d.\n", new_freq);

	if (new_freq == isp->sw_contex.running_freq) {
		dev_dbg(isp->dev, "ignoring DFS target freq.\n");
		return 0;
	}
	ret = write_target_freq_to_hw(isp, new_freq);
	if (!ret)
		isp->sw_contex.running_freq = new_freq;
	return ret;
}

/*
 * reset and restore ISP
 */
int atomisp_reset(struct atomisp_device *isp)
{
	/* Reset ISP by power-cycling it */
	int ret = 0;

	dev_dbg(isp->dev, "%s\n", __func__);
	atomisp_css_suspend();
	ret = pm_runtime_put_sync(isp->dev);
	if (ret < 0) {
		dev_err(isp->dev, "can not disable ISP power\n");
	} else {
		ret = pm_runtime_get_sync(isp->dev);
		if (ret < 0)
			v4l2_err(&atomisp_dev, "can not enable ISP power\n");
	}
	atomisp_css_resume(isp);
	return ret;
}

/*
 * interrupt enable/disable functions
 */
static void enable_isp_irq(enum hrt_isp_css_irq irq, bool enable)
{
	if (enable) {
		irq_enable_channel(IRQ0_ID, irq);
		/*sh_css_hrt_irq_enable(irq, true, false);*/
		switch (irq) { /*We only have sp interrupt right now*/
		case hrt_isp_css_irq_sp:
			/*sh_css_hrt_irq_enable_sp(true);*/
			cnd_sp_irq_enable(SP0_ID, true);
			break;
		default:
			break;
		}

	} else {
		/*sh_css_hrt_irq_disable(irq);*/
		irq_disable_channel(IRQ0_ID, irq);
		switch (irq) {
		case hrt_isp_css_irq_sp:
			/*sh_css_hrt_irq_enable_sp(false);*/
			cnd_sp_irq_enable(SP0_ID, false);
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
	irq_clear_all(IRQ0_ID);
}

void atomisp_msi_irq_init(struct atomisp_device *isp, struct pci_dev *dev)
{
	u32 msg32;
	u16 msg16;

	pci_read_config_dword(dev, PCI_MSI_CAPID, &msg32);
	msg32 |= 1 << MSI_ENABLE_BIT;
	pci_write_config_dword(dev, PCI_MSI_CAPID, msg32);

	msg32 = (1 << INTR_IER) | (1 << INTR_IIR);
	pci_write_config_dword(dev, PCI_INTERRUPT_CTRL, msg32);

	pci_read_config_word(dev, PCI_COMMAND, &msg16);
	msg16 |= (PCI_COMMAND_MEMORY |
		  PCI_COMMAND_MASTER |
		  PCI_COMMAND_INTX_DISABLE);
	pci_write_config_word(dev, PCI_COMMAND, msg16);
}

void atomisp_msi_irq_uninit(struct atomisp_device *isp, struct pci_dev *dev)
{
	u32 msg32;
	u16 msg16;

	pci_read_config_dword(dev, PCI_MSI_CAPID, &msg32);
	msg32 &=  ~(1 << MSI_ENABLE_BIT);
	pci_write_config_dword(dev, PCI_MSI_CAPID, msg32);

	msg32 = 0x0;
	pci_write_config_dword(dev, PCI_INTERRUPT_CTRL, msg32);

	pci_read_config_word(dev, PCI_COMMAND, &msg16);
	msg16 &= ~(PCI_COMMAND_MASTER |
		   PCI_COMMAND_INTX_DISABLE);
	pci_write_config_word(dev, PCI_COMMAND, msg16);
}

static void atomisp_sof_event(struct atomisp_device *isp)
{
	struct v4l2_event event = {0};

	event.type = V4L2_EVENT_FRAME_SYNC;
	event.u.frame_sync.frame_sequence = atomic_read(&isp->sof_count);

	v4l2_event_queue(isp->isp_subdev.subdev.devnode, &event);
}

static void print_csi_rx_errors(struct atomisp_device *isp)
{
	u32 infos = 0;

	atomisp_css_rx_get_irq_info(&infos);

	dev_err(isp->dev, "CSI Receiver errors:\n");
	if (infos & SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		dev_err(isp->dev, "  buffer overrun");
	if (infos & CSS_RX_IRQ_INFO_ERR_SOT)
		dev_err(isp->dev, "  start-of-transmission error");
	if (infos & CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		dev_err(isp->dev, "  start-of-transmission sync error");
	if (infos & CSS_RX_IRQ_INFO_ERR_CONTROL)
		dev_err(isp->dev, "  control error");
	if (infos & CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		dev_err(isp->dev, "  2 or more ECC errors");
	if (infos & CSS_RX_IRQ_INFO_ERR_CRC)
		dev_err(isp->dev, "  CRC mismatch");
	if (infos & CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		dev_err(isp->dev, "  unknown error");
	if (infos & CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		dev_err(isp->dev, "  frame sync error");
	if (infos & CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		dev_err(isp->dev, "  frame data error");
	if (infos & CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		dev_err(isp->dev, "  data timeout");
	if (infos & CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		dev_err(isp->dev, "  unknown escape command entry");
	if (infos & CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		dev_err(isp->dev, "  line sync error");
}

/* interrupt handling function*/
irqreturn_t atomisp_isr(int irq, void *dev)
{
	u32 msg_ret;
	struct atomisp_device *isp = (struct atomisp_device *)dev;
	unsigned int irq_infos = 0;
	unsigned long flags;
	int err;

	err = atomisp_css_irq_translate(isp, &irq_infos);
	if (err)
		return IRQ_NONE;

	dev_dbg(isp->dev, "irq:0x%x\n", irq_infos);

	/* Clear irq reg at PENWELL B0 */
	pci_read_config_dword(isp->pdev, PCI_INTERRUPT_CTRL, &msg_ret);
	msg_ret |= 1 << INTR_IIR;
	pci_write_config_dword(isp->pdev, PCI_INTERRUPT_CTRL, msg_ret);

	spin_lock_irqsave(&isp->lock, flags);
	if (isp->streaming != ATOMISP_DEVICE_STREAMING_ENABLED)
		goto out_nowake;

	if (irq_infos & CSS_IRQ_INFO_CSS_RECEIVER_SOF) {
		atomic_inc(&isp->sof_count);
		atomisp_sof_event(isp);
		irq_infos &= ~CSS_IRQ_INFO_CSS_RECEIVER_SOF;

		/* If sequence_temp and sequence are the same
		 * there where no frames lost so we can increase sequence_temp.
		 * If not then processing of frame is still in progress and
		 * driver needs to keep old sequence_temp value.
		 * NOTE: There is assumption here that ISP will not start
		 * processing next frame from sensor before old one is
		 * completely done. */
		if (atomic_read(&isp->sequence) == atomic_read(
					&isp->sequence_temp))
			atomic_set(&isp->sequence_temp,
					atomic_read(&isp->sof_count));

		/* signal streamon after delayed init is done */
		if (isp->delayed_init == ATOMISP_DELAYED_INIT_WORK_DONE) {
			isp->delayed_init = ATOMISP_DELAYED_INIT_DONE;
			complete(&isp->init_done);
		}
	}

	if (irq_infos & CSS_IRQ_INFO_EVENTS_READY)
		atomic_set(&isp->sequence, atomic_read(&isp->sequence_temp));

#if defined(CONFIG_ISP2400) || defined(CONFIG_ISP2400B0)
	if ((irq_infos & CSS_IRQ_INFO_INPUT_SYSTEM_ERROR) ||
		(irq_infos & CSS_IRQ_INFO_IF_ERROR)) {
#else
	if (irq_infos & CSS_IRQ_INFO_CSS_RECEIVER_ERROR) {
#endif
		/* handle mipi receiver error */
		u32 rx_infos;

		print_csi_rx_errors(isp);
		atomisp_css_rx_get_irq_info(&rx_infos);
		atomisp_css_rx_clear_irq_info(rx_infos);
		/* TODO: handle CSS_RX_IRQ_INFO_BUFFER_OVERRUN */
	}

#ifndef CONFIG_VIDEO_ATOMISP_CSS20
	if (irq_infos & CSS_IRQ_INFO_INVALID_FIRST_FRAME) {
		isp->sw_contex.invalid_frame = 1;
		isp->sw_contex.invalid_vf_frame = 1;
		isp->sw_contex.invalid_s3a = 1;
		isp->sw_contex.invalid_dis = 1;
	}
#endif

	spin_unlock_irqrestore(&isp->lock, flags);

	return IRQ_WAKE_THREAD;

out_nowake:
	spin_unlock_irqrestore(&isp->lock, flags);

	return IRQ_HANDLED;
}

/*
 * Background:
 * The IUNITPHY register CSI_CONTROL bit definition was changed since PNW C0.
 * For PNW A0 and B0, CSI4_TERM_EN_COUNT is bit 23:20 (4 bits).
 * Starting from PWN C0, including all CLV and CLV+ steppings,
 * CSI4_TERM_EN_COUNT is bit 30:24 (7 bits).
 *
 * ------------------------------------------
 * Silicon	Stepping	PCI revision
 * Penwell	A0		0x00
 * Penwell	B0		0x04
 * Penwell	C0		0x06
 * Penwell	D0		0x06
 * Penwell	D1		0x06
 * Penwell	D2		0x06
 * Cloverview	A0		0x06
 * Cloverview	B0		0x05
 * Cloverview	C0		0x04
 * Cloverview+	A0		0x08
 * Cloverview+	B0		0x0C
 *
 */

#define TERM_EN_COUNT_1LANE_OFFSET		16	/* bit 22:16 */
#define TERM_EN_COUNT_1LANE_MASK		0x7f0000
#define TERM_EN_COUNT_4LANE_OFFSET		24	/* bit 30:24 */
#define TERM_EN_COUNT_4LANE_MASK		0x7f000000
#define TERM_EN_COUNT_4LANE_PWN_B0_OFFSET	20	/* bit 23:20 */
#define TERM_EN_COUNT_4LANE_PWN_B0_MASK		0xf00000

void atomisp_set_term_en_count(struct atomisp_device *isp)
{
	uint32_t val;
	int pwn_b0 = 0;

	/* For MRFLD, there is no Tescape-clock cycles control. */
	if (IS_ISP2400(isp))
		return;

	if (isp->pdev->device == 0x0148 && isp->pdev->revision < 0x6 &&
		__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_PENWELL)
		pwn_b0 = 1;

	val = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL);

	/* set TERM_EN_COUNT_1LANE to 0xf */
	val &= ~TERM_EN_COUNT_1LANE_MASK;
	val |= 0xf << TERM_EN_COUNT_1LANE_OFFSET;

	/* set TERM_EN_COUNT_4LANE to 0xf */
	val &= pwn_b0 ? ~TERM_EN_COUNT_4LANE_PWN_B0_MASK :
				~TERM_EN_COUNT_4LANE_MASK;
	val |= 0xf << (pwn_b0 ? TERM_EN_COUNT_4LANE_PWN_B0_OFFSET :
				TERM_EN_COUNT_4LANE_OFFSET);

	intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL, val);
}

void atomisp_clear_css_buffer_counters(struct atomisp_device *isp)
{
	memset(isp->s3a_bufs_in_css, 0, sizeof(isp->s3a_bufs_in_css));
	isp->dis_bufs_in_css = 0;
	isp->isp_subdev.video_out_capture.buffers_in_css = 0;
	isp->isp_subdev.video_out_vf.buffers_in_css = 0;
	isp->isp_subdev.video_out_preview.buffers_in_css = 0;
}

bool atomisp_buffers_queued(struct atomisp_device *isp)
{
	return isp->isp_subdev.video_out_capture.buffers_in_css ||
		isp->isp_subdev.video_out_vf.buffers_in_css ||
		isp->isp_subdev.video_out_preview.buffers_in_css ?
		    true : false;
}

/* 0x100000 is the start of dmem inside SP */
#define SP_DMEM_BASE	0x100000

void dump_sp_dmem(struct atomisp_device *isp, unsigned int addr,
		  unsigned int size)
{
	unsigned int data = 0;
	unsigned int size32 = DIV_ROUND_UP(size, sizeof(u32));

	dev_dbg(isp->dev, "atomisp_io_base:%p\n", atomisp_io_base);
	dev_dbg(isp->dev, "%s, addr:0x%x, size: %d, size32: %d\n", __func__,
			addr, size, size32);
	if (size32 * 4 + addr > 0x4000) {
		dev_err(isp->dev, "illegal size (%d) or addr (0x%x)\n",
				size32, addr);
		return;
	}
	addr += SP_DMEM_BASE;
	do {
		data = _hrt_master_port_uload_32(addr);

		dev_dbg(isp->dev, "%s, \t [0x%x]:0x%x\n", __func__, addr, data);
		addr += sizeof(unsigned int);
		size32 -= 1;
	} while (size32 > 0);
}

static struct videobuf_buffer *atomisp_css_frame_to_vbuf(
	struct atomisp_video_pipe *pipe, struct sh_css_frame *frame)
{
	struct videobuf_vmalloc_memory *vm_mem;
	struct sh_css_frame *handle;
	int i;

	for (i = 0; pipe->capq.bufs[i]; i++) {
		vm_mem = pipe->capq.bufs[i]->priv;
		handle = vm_mem->vaddr;
		if (handle && handle->data == frame->data)
			return pipe->capq.bufs[i];
	}

	return NULL;
}

static void get_buf_timestamp(struct timeval *tv)
{
	struct timespec ts;
	ktime_get_ts(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
}

static void atomisp_flush_video_pipe(struct atomisp_device *isp,
				     struct atomisp_video_pipe *pipe)
{
	unsigned long irqflags;
	int i;

	if (!pipe->users)
		return;

	for (i = 0; pipe->capq.bufs[i]; i++) {
		spin_lock_irqsave(&pipe->irq_lock, irqflags);
		if (pipe->capq.bufs[i]->state == VIDEOBUF_ACTIVE ||
		    pipe->capq.bufs[i]->state == VIDEOBUF_QUEUED) {
			get_buf_timestamp(&pipe->capq.bufs[i]->ts);
			pipe->capq.bufs[i]->field_count =
				atomic_read(&isp->sequence) << 1;
			pipe->capq.bufs[i]->state = VIDEOBUF_ERROR;
			wake_up(&pipe->capq.bufs[i]->done);
			dev_dbg(isp->dev, "release buffers on device %s\n",
				pipe->vdev.name);
			if (pipe->capq.bufs[i]->state == VIDEOBUF_QUEUED)
				list_del_init(&pipe->capq.bufs[i]->queue);
		}
		spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
	}
}

/* Returns queued buffers back to video-core */
void atomisp_flush_bufs_and_wakeup(struct atomisp_device *isp)
{
	atomisp_flush_video_pipe(isp, &isp->isp_subdev.video_out_capture);
	atomisp_flush_video_pipe(isp, &isp->isp_subdev.video_out_vf);
	atomisp_flush_video_pipe(isp, &isp->isp_subdev.video_out_preview);
}


/* find atomisp_video_pipe with css pipe id, buffer type and atomisp run_mode */
static struct atomisp_video_pipe *__atomisp_get_pipe(struct atomisp_device *isp,
		enum sh_css_pipe_id css_pipe_id,
		enum sh_css_buffer_type buf_type)
{
	/* video is same in online as in continuouscapture mode */
	if (!isp->isp_subdev.enable_vfpp->val) {
		return &isp->isp_subdev.video_out_capture;
	} else if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		if (buf_type == SH_CSS_BUFFER_TYPE_OUTPUT_FRAME)
			return &isp->isp_subdev.video_out_capture;
		return &isp->isp_subdev.video_out_preview;
	} else if (buf_type == SH_CSS_BUFFER_TYPE_OUTPUT_FRAME) {
		if (css_pipe_id == SH_CSS_PREVIEW_PIPELINE)
			return &isp->isp_subdev.video_out_preview;
		return &isp->isp_subdev.video_out_capture;
	/* statistic buffers are needed only in css capture & preview pipes */
	} else if (buf_type == SH_CSS_BUFFER_TYPE_3A_STATISTICS ||
		   buf_type == SH_CSS_BUFFER_TYPE_DIS_STATISTICS) {
		if (css_pipe_id == SH_CSS_PREVIEW_PIPELINE)
			return &isp->isp_subdev.video_out_preview;
		return &isp->isp_subdev.video_out_capture;
	}
	return &isp->isp_subdev.video_out_vf;
}

static void atomisp_buf_done(struct atomisp_device *isp, int error,
			     enum atomisp_css_buffer_type buf_type,
			     enum atomisp_css_pipe_id css_pipe_id,
			     bool q_buffers)
{
	struct videobuf_buffer *vb = NULL;
	struct atomisp_video_pipe *pipe = NULL;
	struct atomisp_css_buffer buffer;
	bool requeue = false;
	int err;
	unsigned long irqflags;
	struct sh_css_frame* frame = NULL;

	if (buf_type != CSS_BUFFER_TYPE_3A_STATISTICS &&
	    buf_type != CSS_BUFFER_TYPE_DIS_STATISTICS &&
	    buf_type != CSS_BUFFER_TYPE_OUTPUT_FRAME &&
	    buf_type != CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME &&
	    buf_type != CSS_BUFFER_TYPE_VF_OUTPUT_FRAME) {
		dev_err(isp->dev, "%s, unsupported buffer type: %d\n",
			__func__, buf_type);
		return;
	}

	memset(&buffer, 0, sizeof(struct atomisp_css_buffer));
	buffer.css_buffer.type = buf_type;
	err = atomisp_css_dequeue_buffer(isp, css_pipe_id, buf_type, &buffer);
	if (err) {
		dev_err(isp->dev,
			"atomisp_css_dequeue_buffer failed: 0x%x\n", err);
		return;
	}

	/* need to know the atomisp pipe for frame buffers */
	pipe = __atomisp_get_pipe(isp, css_pipe_id, buf_type);
	if (pipe == NULL) {
		dev_err(isp->dev, "error getting atomisp pipe\n");
		return;
	}

	switch (buf_type) {
		case CSS_BUFFER_TYPE_3A_STATISTICS:
			/* ignore error in case of 3a statistics for now */
			if (isp->sw_contex.invalid_s3a) {
				requeue = true;
				isp->sw_contex.invalid_s3a = 0;
				break;
			}
			/* update the 3A data to ISP context */
			if (isp->params.s3a_output_buf &&
			    isp->params.s3a_output_bytes && !error) {
				/* To avoid racing with atomisp_3a_stat() */
				err = atomisp_css_get_3a_statistics(isp,
								    &buffer);
				if (!err)
					isp->params.s3a_buf_data_valid = true;
				else
					dev_err(isp->dev,
						"get 3a statistics failed, not enough memory\n");
			}
			isp->s3a_bufs_in_css[css_pipe_id]--;
			break;
		case CSS_BUFFER_TYPE_DIS_STATISTICS:
			/* ignore error in case of dis statistics for now */
			if (isp->sw_contex.invalid_dis) {
				requeue = true;
				isp->sw_contex.invalid_dis = 0;
				break;
			}
			if (isp->params.dis_ver_proj_bytes &&
			    isp->params.dis_ver_proj_buf &&
			    isp->params.dis_hor_proj_buf &&
			    isp->params.dis_hor_proj_bytes &&
			    !error) {
				/* To avoid racing with atomisp_get_dis_stat()*/
				atomisp_css_get_dis_statistics(isp, &buffer);

				isp->params.dis_proj_data_valid = true;
			}
			isp->dis_bufs_in_css--;
			break;
		case CSS_BUFFER_TYPE_VF_OUTPUT_FRAME:
			if (isp->sw_contex.invalid_vf_frame) {
				error = true;
				isp->sw_contex.invalid_vf_frame = 0;
				dev_dbg(isp->dev, "%s css has marked this "
					"vf frame as invalid\n", __func__);
			}

			pipe->buffers_in_css--;
			frame = buffer.css_buffer.data.frame;
			if (isp->params.flash_state == ATOMISP_FLASH_ONGOING) {
				if (frame->flash_state
				    == CSS_FRAME_FLASH_STATE_PARTIAL)
					dev_dbg(isp->dev, "%s thumb partially "
						"flashed\n", __func__);
				else if (frame->flash_state
					 == CSS_FRAME_FLASH_STATE_FULL)
					dev_dbg(isp->dev, "%s thumb completely "
						"flashed\n", __func__);
				else
					dev_dbg(isp->dev, "%s thumb no flash "
						"in this frame\n", __func__);
			}
			vb = atomisp_css_frame_to_vbuf(pipe, frame);
			WARN_ON(!vb);
			break;
		case CSS_BUFFER_TYPE_OUTPUT_FRAME:
			if (isp->sw_contex.invalid_frame) {
				error = true;
				isp->sw_contex.invalid_frame = 0;
				dev_dbg(isp->dev, "%s css has marked this "
					"frame as invalid\n", __func__);
			}
			pipe->buffers_in_css--;
			frame = buffer.css_buffer.data.frame;
			vb = atomisp_css_frame_to_vbuf(pipe, frame);
			if (!vb) {
				WARN_ON(1);
				break;
			}

			if (isp->params.flash_state == ATOMISP_FLASH_ONGOING) {
				if (frame->flash_state
				    == CSS_FRAME_FLASH_STATE_PARTIAL) {
					isp->frame_status[vb->i] =
						ATOMISP_FRAME_STATUS_FLASH_PARTIAL;
					dev_dbg(isp->dev,
						 "%s partially flashed\n",
						 __func__);
				} else if (frame->flash_state
					   == CSS_FRAME_FLASH_STATE_FULL) {
					isp->frame_status[vb->i] =
						ATOMISP_FRAME_STATUS_FLASH_EXPOSED;
					isp->params.num_flash_frames--;
					dev_dbg(isp->dev,
						 "%s completely flashed\n",
						 __func__);
				} else {
					isp->frame_status[vb->i] =
						ATOMISP_FRAME_STATUS_OK;
					dev_dbg(isp->dev,
						 "%s no flash in this frame\n",
						 __func__);
				}

				/* Check if flashing sequence is done */
				if (isp->frame_status[vb->i] == ATOMISP_FRAME_STATUS_FLASH_EXPOSED)
					isp->params.flash_state = ATOMISP_FLASH_DONE;
			} else {
				isp->frame_status[vb->i] = ATOMISP_FRAME_STATUS_OK;
			}

			isp->params.last_frame_status = isp->frame_status[vb->i];

			break;
		default:
			break;
	}
	if (vb) {
		get_buf_timestamp(&vb->ts);
		vb->field_count = atomic_read(&isp->sequence) << 1;
		/*mark videobuffer done for dequeue*/
		spin_lock_irqsave(&pipe->irq_lock, irqflags);
		vb->state = !error ? VIDEOBUF_DONE : VIDEOBUF_ERROR;
		spin_unlock_irqrestore(&pipe->irq_lock, irqflags);

		/*
		 * Frame capture done, wake up any process block on
		 * current active buffer
		 * possibly hold by videobuf_dqbuf()
		 */
		wake_up(&vb->done);
	}

	/*
	 * Requeue should only be done for 3a and dis buffers.
	 * Queue/dequeue order will change if driver recycles image buffers.
	 */
	if (requeue) {
		err = atomisp_css_queue_buffer(isp, css_pipe_id,
						buf_type, &buffer);
		if (err)
			dev_err(isp->dev, "%s, q to css fails: %d\n",
					__func__, err);
		return;
	}
	if (!error && q_buffers)
		atomisp_qbuffers_to_css(isp);
}

void atomisp_delayed_init_work(struct work_struct *work)
{
	struct atomisp_device *isp = container_of(work,
						  struct atomisp_device,
						  delayed_init_work);
	sh_css_allocate_continuous_frames(false);
	sh_css_update_continuous_frames();
	isp->delayed_init = ATOMISP_DELAYED_INIT_WORK_DONE;
}


void atomisp_wdt_work(struct work_struct *work)
{
	struct atomisp_device *isp = container_of(work, struct atomisp_device,
						  wdt_work);
	char debug_context[64];
	enum sh_css_pipe_id css_pipe_id;
	int ret;

	dev_err(isp->dev, "timeout %d of %d\n",
		atomic_read(&isp->wdt_count) + 1,
		ATOMISP_ISP_MAX_TIMEOUT_COUNT);

	mutex_lock(&isp->mutex);
	if (isp->streaming != ATOMISP_DEVICE_STREAMING_ENABLED) {
		mutex_unlock(&isp->mutex);
		return;
	}

	switch (atomic_inc_return(&isp->wdt_count)) {
	case ATOMISP_ISP_MAX_TIMEOUT_COUNT:
		atomisp_clear_css_buffer_counters(isp);

		atomic_set(&isp->wdt_count, 0);

		atomisp_flush_bufs_and_wakeup(isp);
		isp->isp_fatal_error = true;

		complete(&isp->init_done);

		break;
	default:
		sh_css_dump_sp_sw_debug_info();
		sh_css_dump_debug_info(debug_context);
		dev_err(isp->dev, "%s, vdev %s buffers in css: %d\n", __func__,
			isp->isp_subdev.video_out_capture.vdev.name,
			isp->isp_subdev.video_out_capture.buffers_in_css);
		dev_err(isp->dev, "%s, vdev %s buffers in css: %d\n", __func__,
			isp->isp_subdev.video_out_vf.vdev.name,
			isp->isp_subdev.video_out_vf.buffers_in_css);
		dev_err(isp->dev, "%s, vdev %s buffers in css: %d\n", __func__,
			isp->isp_subdev.video_out_preview.vdev.name,
			isp->isp_subdev.video_out_preview.buffers_in_css);
		dev_err(isp->dev, "%s, s3a buffers in css preview pipe: %d\n",
			__func__,
			isp->s3a_bufs_in_css[SH_CSS_PREVIEW_PIPELINE]);
		dev_err(isp->dev, "%s, s3a buffers in css capture pipe: %d\n",
			__func__,
			isp->s3a_bufs_in_css[SH_CSS_CAPTURE_PIPELINE]);
		dev_err(isp->dev, "%s, s3a buffers in css video pipe: %d\n",
			__func__, isp->s3a_bufs_in_css[SH_CSS_VIDEO_PIPELINE]);
		dev_err(isp->dev, "%s, dis buffers in css: %d\n",
			__func__, isp->dis_bufs_in_css);
		/*sh_css_dump_sp_state();*/
		/*sh_css_dump_isp_state();*/

		if (!isp->sw_contex.file_input)
			atomisp_css_irq_enable(isp,
					CSS_IRQ_INFO_CSS_RECEIVER_SOF, false);

		if (isp->delayed_init == ATOMISP_DELAYED_INIT_QUEUED)
			cancel_work_sync(&isp->delayed_init_work);

		complete(&isp->init_done);
		isp->delayed_init = ATOMISP_DELAYED_INIT_NOT_QUEUED;

		css_pipe_id = atomisp_get_css_pipe_id(isp);
		switch (css_pipe_id) {
		case SH_CSS_PREVIEW_PIPELINE:
			sh_css_preview_stop();
			break;
		case SH_CSS_VIDEO_PIPELINE:
			sh_css_video_stop();
			break;
		case SH_CSS_CAPTURE_PIPELINE:
			/* fall through */
		default:
			sh_css_capture_stop();
			break;
		}
		atomisp_acc_unload_extensions(isp);

		/* clear irq */
		enable_isp_irq(hrt_isp_css_irq_sp, false);
		clear_isp_irq(hrt_isp_css_irq_sp);
		/*
		 * TODO: do we need to reset any other interrupts,
		 * i.e hrt_isp_css_irq_sw_1 or hrt_isp_css_irq_sw_2?
		 */
		/* stream off sensor */
		ret = v4l2_subdev_call(
			isp->inputs[isp->input_curr].camera, video,
			s_stream, 0);
		if (ret)
			dev_warn(isp->dev,
				 "can't stop streaming on sensor!\n");

		/* reset ISP and restore its state */
		isp->isp_timeout = true;
		atomisp_reset(isp);
		isp->isp_timeout = false;

		atomisp_clear_css_buffer_counters(isp);
		if (atomisp_acc_load_extensions(isp) < 0)
			dev_err(isp->dev, "acc extension failed to reload\n");

		atomisp_css_start(isp, css_pipe_id, true);

		if (!isp->sw_contex.file_input) {
			atomisp_css_irq_enable(isp,
					CSS_IRQ_INFO_CSS_RECEIVER_SOF, true);

			atomisp_set_term_en_count(isp);

			if (IS_ISP2400(isp) &&
			atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_AUTO) < 0)
				dev_dbg(isp->dev, "dfs failed!\n");
		} else {
			if (IS_ISP2400(isp) &&
			atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_MAX) < 0)
				dev_dbg(isp->dev, "dfs failed!\n");
		}

		ret = v4l2_subdev_call(
			isp->inputs[isp->input_curr].camera, video,
			s_stream, 1);
		if (ret)
			dev_warn(isp->dev,
				 "can't start streaming on sensor!\n");

		if (isp->isp_subdev.continuous_mode->val &&
		    isp->isp_subdev.run_mode->val != ATOMISP_RUN_MODE_VIDEO &&
		    isp->delayed_init == ATOMISP_DELAYED_INIT_NOT_QUEUED) {
			INIT_COMPLETION(isp->init_done);
			isp->delayed_init = ATOMISP_DELAYED_INIT_QUEUED;
			queue_work(isp->delayed_init_workq,
				   &isp->delayed_init_work);
		}
		/*
		 * dequeueing buffers is not needed. CSS will recycle
		 * buffers that it has.
		 */
		atomisp_flush_bufs_and_wakeup(isp);
		dev_err(isp->dev, "timeout recovery handling done\n");
	}

	mutex_unlock(&isp->mutex);
}

void atomisp_wdt(unsigned long isp_addr)
{
	struct atomisp_device *isp = (struct atomisp_device *)isp_addr;

	queue_work(isp->wdt_work_queue, &isp->wdt_work);
}

void atomisp_setup_flash(struct atomisp_device *isp)
{
	if (isp->params.flash_state != ATOMISP_FLASH_REQUESTED &&
	    isp->params.flash_state != ATOMISP_FLASH_DONE)
		return;

	if (isp->params.num_flash_frames) {
		struct v4l2_control ctrl;

		/* make sure the timeout is set before setting flash mode */
		ctrl.id = V4L2_CID_FLASH_TIMEOUT;
		ctrl.value = FLASH_TIMEOUT;

		if (v4l2_subdev_call(isp->flash, core, s_ctrl, &ctrl)) {
			dev_err(isp->dev, "flash timeout configure failed\n");
			return;
		}
		sh_css_request_flash();
		isp->params.flash_state = ATOMISP_FLASH_ONGOING;
	} else {
		/* Flashing all frames is done */
		isp->params.flash_state = ATOMISP_FLASH_IDLE;
	}
}

irqreturn_t atomisp_isr_thread(int irq, void *isp_ptr)
{
	struct atomisp_device *isp = isp_ptr;
	struct atomisp_css_event current_event;
	unsigned long flags;
	bool frame_done_found = false;
	bool css_pipe_done = false;
	bool reset_wdt_timer = false;
	DEFINE_KFIFO(events, struct atomisp_css_event, ATOMISP_CSS_EVENTS_MAX);

	dev_dbg(isp->dev, ">%s\n", __func__);
	mutex_lock(&isp->mutex);

	spin_lock_irqsave(&isp->lock, flags);
	if (isp->streaming != ATOMISP_DEVICE_STREAMING_ENABLED) {
		spin_unlock_irqrestore(&isp->lock, flags);
		goto out;
	}
	spin_unlock_irqrestore(&isp->lock, flags);

	while (sh_css_dequeue_event(&current_event.pipe,
				    &current_event.event) == sh_css_success) {
		switch (current_event.event) {
		case CSS_EVENT_PIPELINE_DONE:
			css_pipe_done = true;
			break;
		case CSS_EVENT_OUTPUT_FRAME_DONE:
		case CSS_EVENT_VF_OUTPUT_FRAME_DONE:
			reset_wdt_timer = true; /* ISP running */
			/* fall through */
		case CSS_EVENT_3A_STATISTICS_DONE:
		case CSS_EVENT_DIS_STATISTICS_DONE:
			break;
		default:
			dev_err(isp->dev, "unknown event 0x%x pipe:%d\n",
				current_event.event, current_event.pipe);
			break;
		}
		kfifo_in(&events, &current_event, 1);
	}

	while (kfifo_out(&events, &current_event, 1)) {
		switch (current_event.event) {
		case CSS_EVENT_OUTPUT_FRAME_DONE:
			frame_done_found = true;
			atomisp_buf_done(isp, 0,
					 SH_CSS_BUFFER_TYPE_OUTPUT_FRAME,
					 current_event.pipe,
					 true);
			break;
		case CSS_EVENT_3A_STATISTICS_DONE:
			atomisp_buf_done(isp, 0,
					 SH_CSS_BUFFER_TYPE_3A_STATISTICS,
					 current_event.pipe,
					 css_pipe_done);
			break;
		case CSS_EVENT_VF_OUTPUT_FRAME_DONE:
			atomisp_buf_done(isp, 0,
					 SH_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME,
					 current_event.pipe,
					 true);
			break;
		case CSS_EVENT_DIS_STATISTICS_DONE:
			atomisp_buf_done(isp, 0,
					 SH_CSS_BUFFER_TYPE_DIS_STATISTICS,
					 current_event.pipe,
					 css_pipe_done);
			break;
		case CSS_EVENT_PIPELINE_DONE:
			break;
		default:
			dev_err(isp->dev, "unhandled css stored event: 0x%x\n",
					current_event.event);
			break;
		}
	}

	if (frame_done_found &&
	    isp->params.css_update_params_needed) {
		atomisp_css_update_isp_params(isp);
		isp->params.css_update_params_needed = false;
		frame_done_found = false;
	}
	atomisp_setup_flash(isp);

	/* If there are no buffers queued then delete wdt timer. */
	if (!atomisp_buffers_queued(isp)) {
		del_timer(&isp->wdt);
	} else {
		/* SOF irq should not reset wdt timer. */
		if (reset_wdt_timer) {
			mod_timer(&isp->wdt, jiffies + isp->wdt_duration);
			atomic_set(&isp->wdt_count, 0);
		}
	}
out:
	mutex_unlock(&isp->mutex);

	if (isp->streaming == ATOMISP_DEVICE_STREAMING_ENABLED
		&& css_pipe_done
		&& isp->sw_contex.file_input)
		v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				video, s_stream, 1);

	dev_dbg(isp->dev, "<%s\n", __func__);

	return IRQ_HANDLED;
}

/*
 * utils for buffer allocation/free
 */

int atomisp_get_frame_pgnr(struct atomisp_device *isp,
			   const struct sh_css_frame *frame, u32 *p_pgnr)
{
	if (!frame) {
		dev_err(isp->dev, "%s: NULL frame pointer ERROR.\n", __func__);
		return -EINVAL;
	}

	*p_pgnr = DIV_ROUND_UP(frame->data_bytes, PAGE_SIZE);
	return 0;
}

/*
 * Get internal fmt according to V4L2 fmt
 */
static enum sh_css_frame_format
v4l2_fmt_to_sh_fmt(u32 fmt)
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

bool atomisp_is_mbuscode_raw(uint32_t code)
{
	return code >= 0x3000 && code < 0x4000;
}

/*
 * ISP features control function
 */

/*
 * Set ISP capture mode based on current settings
 */
static void atomisp_update_capture_mode(struct atomisp_device *isp)
{
	if (isp->params.low_light)
		atomisp_css_capture_set_mode(isp, CSS_CAPTURE_MODE_LOW_LIGHT);
	else if (isp->params.gdc_cac_en)
		atomisp_css_capture_set_mode(isp, CSS_CAPTURE_MODE_ADVANCED);
	else
		atomisp_css_capture_set_mode(isp, CSS_CAPTURE_MODE_PRIMARY);
}

/*
 * Function to enable/disable lens geometry distortion correction (GDC) and
 * chromatic aberration correction (CAC)
 */
int atomisp_gdc_cac(struct atomisp_device *isp, int flag, __s32 * value)
{
	if (flag == 0) {
		*value = isp->params.gdc_cac_en;
		return 0;
	}

	isp->params.gdc_cac_en = !!*value;
	if (isp->params.gdc_cac_en) {
		sh_css_set_morph_table(
				isp->inputs[isp->input_curr].morph_table);
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
 * Function to enable/disable extra noise reduction (XNR) in low light
 * condition
 */
int atomisp_xnr(struct atomisp_device *isp, int flag, int *xnr_enable)
{
	if (flag == 0) {
		*xnr_enable = isp->params.xnr_en;
		return 0;
	}

	sh_css_capture_enable_xnr(!!*xnr_enable);

	return 0;
}

/*
 * Function to configure bayer noise reduction
 */
int atomisp_nr(struct atomisp_device *isp, int flag,
	       struct atomisp_nr_config *arg)
{
	if (flag == 0) {
		/* Get nr config from current setup */
		const struct sh_css_nr_config *nr_config;
		sh_css_get_nr_config(&nr_config);
		memcpy(arg, nr_config, sizeof(*arg));
	} else {
		/* Set nr config to isp parameters */
		memcpy(&isp->params.nr_config, arg,
			sizeof(struct sh_css_nr_config));
		sh_css_set_nr_config(&isp->params.nr_config);
		isp->params.css_update_params_needed = true;
	}
	return 0;
}

/*
 * Function to configure temporal noise reduction (TNR)
 */
int atomisp_tnr(struct atomisp_device *isp, int flag,
		struct atomisp_tnr_config *config)
{
	/* Get tnr config from current setup */
	if (flag == 0) {
		/* Get tnr config from current setup */
		memcpy(config, &isp->params.tnr_config, sizeof(*config));
	} else {
		/* Set tnr config to isp parameters */
		memcpy(&isp->params.tnr_config, config,
			sizeof(struct sh_css_tnr_config));
		sh_css_set_tnr_config(&isp->params.tnr_config);
		isp->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to configure black level compensation
 */
int atomisp_black_level(struct atomisp_device *isp, int flag,
			struct atomisp_ob_config *config)
{
	if (flag == 0) {
		/* Get ob config from current setup */
		const struct sh_css_ob_config *ob_config;
		sh_css_get_ob_config(&ob_config);
		memcpy(config, ob_config, sizeof(*config));
	} else {
		/* Set ob config to isp parameters */
		memcpy(&isp->params.ob_config, config,
			sizeof(struct sh_css_ob_config));
		sh_css_set_ob_config(&isp->params.ob_config);
		isp->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to configure edge enhancement
 */
int atomisp_ee(struct atomisp_device *isp, int flag,
	       struct atomisp_ee_config *config)
{
	if (flag == 0) {
		/* Get ee config from current setup */
		const struct sh_css_ee_config *ee_config;
		sh_css_get_ee_config(&ee_config);
		memcpy(config, ee_config, sizeof(*config));
	} else {
		/* Set ee config to isp parameters */
		memcpy(&isp->params.ee_config, config,
		       sizeof(isp->params.ee_config));
		sh_css_set_ee_config(&isp->params.ee_config);
		isp->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to update Gamma table for gamma, brightness and contrast config
 */
int atomisp_gamma(struct atomisp_device *isp, int flag,
		  struct atomisp_gamma_table *config)
{
	if (flag == 0) {
		/* Get gamma table from current setup */
		const struct sh_css_gamma_table *tab;
		sh_css_get_gamma_table(&tab);
		memcpy(config, tab, sizeof(*config));
	} else {
		/* Set gamma table to isp parameters */
		memcpy(&isp->params.gamma_table, config,
		       sizeof(isp->params.gamma_table));
		sh_css_set_gamma_table(&isp->params.gamma_table);
	}

	return 0;
}

/*
 * Function to update Ctc table for Chroma Enhancement
 */
int atomisp_ctc(struct atomisp_device *isp, int flag,
		struct atomisp_ctc_table *config)
{
	if (flag == 0) {
		/* Get ctc table from current setup */
		const struct sh_css_ctc_table *tab;
		sh_css_get_ctc_table(&tab);
		memcpy(config, tab, sizeof(*config));
	} else {
		/* Set ctc table to isp parameters */
		memcpy(&isp->params.ctc_table, config,
			sizeof(isp->params.ctc_table));
		sh_css_set_ctc_table(&isp->params.ctc_table);
	}

	return 0;
}

/*
 * Function to update gamma correction parameters
 */
int atomisp_gamma_correction(struct atomisp_device *isp, int flag,
	struct atomisp_gc_config *config)
{
	if (flag == 0) {
		/* Get gamma correction params from current setup */
		memcpy(config, &isp->params.gc_config, sizeof(*config));
	} else {
		/* Set gamma correction params to isp parameters */
		memcpy(&isp->params.gc_config, config, sizeof(*config));
		sh_css_set_gc_config(&isp->params.gc_config);
		isp->params.css_update_params_needed = true;
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
	if (isp->raw_output_frame) {
		sh_css_frame_free(isp->raw_output_frame);
		isp->raw_output_frame = NULL;
	}
}

void atomisp_free_3a_dis_buffers(struct atomisp_device *isp)
{
	struct atomisp_s3a_buf *s3a_buf, *_s3a_buf;
	struct atomisp_dis_buf *dis_buf, *_dis_buf;

	/* 3A statistics use vmalloc, DIS use kmalloc */
	if (isp->params.s3a_output_buf)
		vfree(isp->params.s3a_output_buf);
	isp->params.s3a_output_buf = NULL;
	isp->params.s3a_output_bytes = 0;
	isp->params.s3a_buf_data_valid = false;

	kfree(isp->params.dis_hor_proj_buf);
	kfree(isp->params.dis_ver_proj_buf);
	kfree(isp->params.dis_hor_coef_buf);
	kfree(isp->params.dis_ver_coef_buf);

	isp->params.dis_hor_proj_buf = NULL;
	isp->params.dis_ver_proj_buf = NULL;
	isp->params.dis_hor_coef_buf = NULL;
	isp->params.dis_ver_coef_buf = NULL;
	isp->params.dis_hor_proj_bytes = 0;
	isp->params.dis_ver_proj_bytes = 0;
	isp->params.dis_hor_coef_bytes = 0;
	isp->params.dis_ver_coef_bytes = 0;
	isp->params.dis_proj_data_valid = false;

	list_for_each_entry_safe(s3a_buf, _s3a_buf, &isp->s3a_stats, list) {
		sh_css_free_stat_buffers(&s3a_buf->s3a_data, NULL);
		list_del(&s3a_buf->list);
		kfree(s3a_buf);
	}

	list_for_each_entry_safe(dis_buf, _dis_buf, &isp->dis_stats, list) {
		sh_css_free_stat_buffers(NULL, &dis_buf->dis_data);
		list_del(&dis_buf->list);
		kfree(dis_buf);
	}
}

static void atomisp_update_grid_info(struct atomisp_device *isp)
{
	int err;
	struct sh_css_grid_info old_info = isp->params.curr_grid_info;
	switch (isp->isp_subdev.run_mode->val) {
	case ATOMISP_RUN_MODE_PREVIEW:
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, preview\n",
			 __func__);
		err = sh_css_preview_get_grid_info(&isp->params.curr_grid_info);
		if(err) {
			dev_err(isp->dev,
				 "sh_css_preview_get_grid_info failed: %d\n",
				 err);
			return;
		}
		break;
	case ATOMISP_RUN_MODE_VIDEO:
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, video\n", __func__);
		err = sh_css_video_get_grid_info(&isp->params.curr_grid_info);
		if (err) {
			dev_err(isp->dev,
				 "sh_css_video_get_grid_info failed: %d\n",
				 err);
			return;
		}
		break;
	default:
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, default(capture)\n",
			 __func__);
		err = sh_css_capture_get_grid_info(&isp->params.curr_grid_info);
		if (err) {
			dev_err(isp->dev,
				 "sh_css_capture_get_grid_info failed: %d\n",
				 err);
			return;
		}
		break;
	}

	/* If the grid info has not changed and the buffers for 3A and
	 * DIS statistics buffers are allocated or buffer size would be zero
	 * then no need to do anything. */
	if ((!memcmp(&old_info, &isp->params.curr_grid_info, sizeof(old_info)) &&
		isp->params.s3a_output_buf && isp->params.dis_hor_coef_buf) ||
		isp->params.curr_grid_info.s3a_grid.width == 0 ||
		isp->params.curr_grid_info.s3a_grid.height == 0)
		return;

	/* We must free all buffers because they no longer match
	   the grid size. */
	atomisp_free_3a_dis_buffers(isp);

	err = atomisp_alloc_css_stat_bufs(isp);
	if (err) {
		dev_err(isp->dev, "stat_buf allocate error\n");
		goto err_3a;
	}

	/* 3A statistics. These can be big, so we use vmalloc. */
	isp->params.s3a_output_bytes =
		isp->params.curr_grid_info.s3a_grid.width *
		isp->params.curr_grid_info.s3a_grid.height *
		sizeof(*isp->params.s3a_output_buf);

	v4l2_dbg(3, dbg_level, &atomisp_dev,
			"isp->params.s3a_output_bytes: %d\n",
			isp->params.s3a_output_bytes);
	isp->params.s3a_output_buf = vmalloc(isp->params.s3a_output_bytes);

	if (isp->params.s3a_output_buf == NULL &&
			isp->params.s3a_output_bytes != 0)
		goto err_3a;

	memset(isp->params.s3a_output_buf, 0, isp->params.s3a_output_bytes);
	isp->params.s3a_buf_data_valid = false;

	/* DIS coefficients. */
	isp->params.dis_hor_coef_bytes =
		isp->params.curr_grid_info.dvs_hor_coef_num *
		SH_CSS_DIS_NUM_COEF_TYPES *
		sizeof(*isp->params.dis_hor_coef_buf);

	isp->params.dis_ver_coef_bytes =
		isp->params.curr_grid_info.dvs_ver_coef_num *
		SH_CSS_DIS_NUM_COEF_TYPES *
		sizeof(*isp->params.dis_ver_coef_buf);

	isp->params.dis_hor_coef_buf =
		kzalloc(isp->params.dis_hor_coef_bytes, GFP_KERNEL);
	if (isp->params.dis_hor_coef_buf == NULL &&
			isp->params.dis_hor_coef_bytes != 0)
		goto err_dis;

	isp->params.dis_ver_coef_buf =
		kzalloc(isp->params.dis_ver_coef_bytes, GFP_KERNEL);
	if (isp->params.dis_ver_coef_buf == NULL &&
			isp->params.dis_ver_coef_bytes != 0)
		goto err_dis;

	/* DIS projections. */
	isp->params.dis_proj_data_valid = false;
	isp->params.dis_hor_proj_bytes =
		isp->params.curr_grid_info.dvs_grid.aligned_height *
		SH_CSS_DIS_NUM_COEF_TYPES *
		sizeof(*isp->params.dis_hor_proj_buf);

	isp->params.dis_ver_proj_bytes =
		isp->params.curr_grid_info.dvs_grid.aligned_width *
		SH_CSS_DIS_NUM_COEF_TYPES *
		sizeof(*isp->params.dis_ver_proj_buf);

	isp->params.dis_hor_proj_buf = kzalloc(isp->params.dis_hor_proj_bytes,
			GFP_KERNEL);
	if (isp->params.dis_hor_proj_buf == NULL &&
			isp->params.dis_hor_proj_bytes != 0)
		goto err_dis;

	isp->params.dis_ver_proj_buf = kzalloc(isp->params.dis_ver_proj_bytes,
			GFP_KERNEL);
	if (isp->params.dis_ver_proj_buf == NULL &&
			isp->params.dis_ver_proj_bytes != 0)
		goto err_dis;

	return;

	/* Failure for 3A buffers does not influence DIS buffers */
err_3a:
	if (isp->params.s3a_output_bytes != 0) {
		/* For SOC sensor happens s3a_output_bytes == 0,
		*  using if condition to exclude false error log */
		dev_err(isp->dev, "Failed allocate memory for 3A statistics\n");
	}
	atomisp_free_3a_dis_buffers(isp);
	return;

err_dis:
	dev_err(isp->dev, "Failed allocate memory for DIS statistics\n");
	atomisp_free_3a_dis_buffers(isp);
}

static void atomisp_curr_user_grid_info(struct atomisp_device *isp,
				    struct atomisp_grid_info *info)
{
	info->isp_in_width          = isp->params.curr_grid_info.isp_in_width;
	info->isp_in_height         = isp->params.curr_grid_info.isp_in_height;
	info->s3a_width             = isp->params.curr_grid_info.s3a_grid.width;
	info->s3a_height            =
		isp->params.curr_grid_info.s3a_grid.height;
	info->s3a_bqs_per_grid_cell =
		isp->params.curr_grid_info.s3a_grid.bqs_per_grid_cell;

	info->dis_width          = isp->params.curr_grid_info.dvs_grid.width;
	info->dis_aligned_width  =
		isp->params.curr_grid_info.dvs_grid.aligned_width;
	info->dis_height         = isp->params.curr_grid_info.dvs_grid.height;
	info->dis_aligned_height =
		isp->params.curr_grid_info.dvs_grid.aligned_height;
	info->dis_bqs_per_grid_cell =
		isp->params.curr_grid_info.dvs_grid.bqs_per_grid_cell;
	info->dis_hor_coef_num      =
		isp->params.curr_grid_info.dvs_hor_coef_num;
	info->dis_ver_coef_num      =
		isp->params.curr_grid_info.dvs_ver_coef_num;
}

static int atomisp_compare_grid(struct atomisp_device *isp,
				struct atomisp_grid_info *atomgrid)
{
	struct atomisp_grid_info tmp = {0};

	atomisp_curr_user_grid_info(isp, &tmp);
	return memcmp(atomgrid, &tmp, sizeof(tmp));
}

/*
 * Function to update Gdc table for gdc
 */
int atomisp_gdc_cac_table(struct atomisp_device *isp, int flag,
			  struct atomisp_morph_table *config)
{
	int ret;
	int i;

	if (flag == 0) {
		/* Get gdc table from current setup */
		const struct sh_css_morph_table *tab;
		sh_css_get_morph_table(&tab);

		config->width = tab->width;
		config->height = tab->height;

		for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			ret = copy_to_user(config->coordinates_x[i],
				tab->coordinates_x[i], tab->height *
				tab->width * sizeof(*tab->coordinates_x[i]));
			if (ret) {
				dev_err(isp->dev,
					"Failed to copy to User for x\n");
				return -EFAULT;
			}
			ret = copy_to_user(config->coordinates_y[i],
				tab->coordinates_y[i], tab->height *
				tab->width * sizeof(*tab->coordinates_y[i]));
			if (ret) {
				dev_err(isp->dev,
					"Failed to copy to User for y\n");
				return -EFAULT;
			}
		}
	} else {
		struct sh_css_morph_table *tab =
			isp->inputs[isp->input_curr].morph_table;

		/* free first if we have one */
		if (tab) {
			sh_css_morph_table_free(tab);
			isp->inputs[isp->input_curr].morph_table = NULL;
		}

		/* allocate new one */
		tab = sh_css_morph_table_allocate(config->width,
						  config->height);

		if (!tab) {
			dev_err(isp->dev, "out of memory\n");
			return -EINVAL;
		}

		for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			ret = copy_from_user(tab->coordinates_x[i],
				config->coordinates_x[i],
				config->height * config->width *
				sizeof(*config->coordinates_x[i]));
			if (ret) {
				dev_err(isp->dev,
				"Failed to copy from User for x, ret %d\n",
				ret);
				sh_css_morph_table_free(tab);
				return -EFAULT;
			}
			ret = copy_from_user(tab->coordinates_y[i],
				config->coordinates_y[i],
				config->height * config->width *
				sizeof(*config->coordinates_y[i]));
			if (ret) {
				dev_err(isp->dev,
				"Failed to copy from User for y, ret is %d\n",
				ret);
				sh_css_morph_table_free(tab);
				return -EFAULT;
			}
		}
		isp->inputs[isp->input_curr].morph_table = tab;
		if (isp->params.gdc_cac_en)
			sh_css_set_morph_table(tab);
	}

	return 0;
}

int atomisp_macc_table(struct atomisp_device *isp, int flag,
		       struct atomisp_macc_config *config)
{
	struct sh_css_macc_table *macc_table;

	switch (config->color_effect) {
	case V4L2_COLORFX_NONE:
		macc_table = &isp->params.macc_table;
		break;
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
		memcpy(&config->table, macc_table,
		       sizeof(struct sh_css_macc_table));
	} else {
		memcpy(macc_table, &config->table,
		       sizeof(struct sh_css_macc_table));
		if (config->color_effect == isp->params.color_effect)
			sh_css_set_macc_table(macc_table);
	}

	return 0;
}

int atomisp_set_dis_vector(struct atomisp_device *isp,
			   struct atomisp_dis_vector *vector)
{
	sh_css_video_set_dis_vector(vector->x, vector->y);

	isp->params.dis_proj_data_valid = false;
	isp->params.css_update_params_needed = true;
	return 0;
}

/*
 * Function to set/get image stablization statistics
 */
int atomisp_get_dis_stat(struct atomisp_device *isp,
			 struct atomisp_dis_statistics *stats)
{
	unsigned long flags;
	int error;

	if (stats->vertical_projections   == NULL ||
	    stats->horizontal_projections == NULL ||
	    isp->params.dis_hor_proj_buf  == NULL ||
	    isp->params.dis_ver_proj_buf  == NULL)
		return -EINVAL;

	/* isp needs to be streaming to get DIS statistics */
	spin_lock_irqsave(&isp->lock, flags);
	if (isp->streaming != ATOMISP_DEVICE_STREAMING_ENABLED) {
		spin_unlock_irqrestore(&isp->lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&isp->lock, flags);

	if (!isp->params.video_dis_en)
		return -EINVAL;

	if (atomisp_compare_grid(isp, &stats->grid_info) != 0)
		/* If the grid info in the argument differs from the current
		   grid info, we tell the caller to reset the grid size and
		   try again. */
		return -EAGAIN;

	if (!isp->params.dis_proj_data_valid)
		return -EBUSY;

	error = copy_to_user(stats->vertical_projections,
			     isp->params.dis_ver_proj_buf,
			     isp->params.dis_ver_proj_bytes);

	error |= copy_to_user(stats->horizontal_projections,
			     isp->params.dis_hor_proj_buf,
			     isp->params.dis_hor_proj_bytes);

	if (error)
		return -EFAULT;

	return 0;
}

int atomisp_set_dis_coefs(struct atomisp_device *isp,
			  struct atomisp_dis_coefficients *coefs)
{
	int error;

	if (coefs->horizontal_coefficients == NULL ||
	    coefs->vertical_coefficients   == NULL ||
	    isp->params.dis_hor_coef_buf   == NULL ||
	    isp->params.dis_ver_coef_buf   == NULL)
		return -EINVAL;

	error = copy_from_user(isp->params.dis_hor_coef_buf,
			       coefs->horizontal_coefficients,
			       isp->params.dis_hor_coef_bytes);
	if (error)
		return -EFAULT;
	error = copy_from_user(isp->params.dis_ver_coef_buf,
			       coefs->vertical_coefficients,
			       isp->params.dis_ver_coef_bytes);
	if (error)
		return -EFAULT;
	sh_css_set_dis_coefficients(isp->params.dis_hor_coef_buf,
				    isp->params.dis_ver_coef_buf);

	isp->params.dis_proj_data_valid = false;

	return 0;
}

/*
 * Function to set/get 3A stat from isp
 */
int atomisp_3a_stat(struct atomisp_device *isp, int flag,
		    struct atomisp_3a_statistics *config)
{
	unsigned long ret;

	if (flag != 0)
		return -EINVAL;

	/* sanity check to avoid writing into unallocated memory. */
	if (isp->params.s3a_output_bytes == 0)
		return -EINVAL;

	if (atomisp_compare_grid(isp, &config->grid_info) != 0) {
		/* If the grid info in the argument differs from the current
		   grid info, we tell the caller to reset the grid size and
		   try again. */
		return -EAGAIN;
	}

	/* This is done in the atomisp_s3a_buf_done() */
	if(!isp->params.s3a_buf_data_valid) {
		dev_err(isp->dev, "3a statistics is not valid.\n");
		return -EAGAIN;
	}

	ret = copy_to_user(config->data,
			   isp->params.s3a_output_buf,
			   isp->params.s3a_output_bytes);
	if (ret) {
		dev_err(isp->dev, "copy to user failed: copied %lu bytes\n",
				ret);
		return -EFAULT;
	}
	return 0;
}

static int __atomisp_set_general_isp_parameters(struct atomisp_device *isp,
					struct atomisp_parameters *arg)
{
	if (arg->wb_config) {
		if (copy_from_user(&isp->params.wb_config, arg->wb_config,
			sizeof(struct sh_css_wb_config)))
			return -EFAULT;
		sh_css_set_wb_config(&isp->params.wb_config);
	}

	if (arg->ob_config) {
		if (copy_from_user(&isp->params.ob_config, arg->ob_config,
			sizeof(struct sh_css_ob_config)))
			return -EFAULT;
		sh_css_set_ob_config(&isp->params.ob_config);
	}

	if (arg->dp_config) {
		if (copy_from_user(&isp->params.dp_config, arg->dp_config,
			sizeof(struct sh_css_dp_config)))
			return -EFAULT;
		sh_css_set_dp_config(&isp->params.dp_config);
	}

	if (arg->de_config) {
		if (copy_from_user(&isp->params.de_config, arg->de_config,
			sizeof(struct sh_css_de_config)))
			return -EFAULT;
		sh_css_set_de_config(&isp->params.de_config);
	}

	if (arg->ce_config) {
		if (copy_from_user(&isp->params.ce_config, arg->ce_config,
			sizeof(struct sh_css_ce_config)))
			return -EFAULT;
		sh_css_set_ce_config(&isp->params.ce_config);
	}

	if (arg->nr_config) {
		if (copy_from_user(&isp->params.nr_config, arg->nr_config,
			sizeof(struct sh_css_nr_config)))
			return -EFAULT;
		sh_css_set_nr_config(&isp->params.nr_config);
	}

	if (arg->ee_config) {
		if (copy_from_user(&isp->params.ee_config, arg->ee_config,
			sizeof(struct sh_css_ee_config)))
			return -EFAULT;
		sh_css_set_ee_config(&isp->params.ee_config);
	}

	if (arg->tnr_config) {
		if (copy_from_user(&isp->params.tnr_config, arg->tnr_config,
			sizeof(struct sh_css_tnr_config)))
			return -EFAULT;
		sh_css_set_tnr_config(&isp->params.tnr_config);
	}

	if (arg->cc_config) {
		if (copy_from_user(&isp->params.cc_config, arg->cc_config,
			sizeof(struct sh_css_cc_config)))
			return -EFAULT;
		sh_css_set_cc_config(&isp->params.cc_config);
	}

	if (arg->macc_config) {
		if (copy_from_user(&isp->params.macc_table,
			&arg->macc_config->table,
			sizeof(struct sh_css_macc_table)))
			return -EFAULT;
		isp->params.color_effect = arg->macc_config->color_effect;
		sh_css_set_macc_table(&isp->params.macc_table);
	}

	if (arg->gamma_table) {
		if (copy_from_user(&isp->params.gamma_table, arg->gamma_table,
			sizeof(isp->params.gamma_table)))
			return -EFAULT;
		sh_css_set_gamma_table(&isp->params.gamma_table);
	}

	if (arg->ctc_table) {
		if (copy_from_user(&isp->params.ctc_table, arg->ctc_table,
			sizeof(isp->params.ctc_table)))
			return -EFAULT;
		sh_css_set_ctc_table(&isp->params.ctc_table);
	}

	/*
	 * TODO/FIXME: No implementation exists for setting the XNR threshold
	 * in CSS. Enable this when CSS implementation is ready.
	 */
	/*
	if (arg->xnr_config)
		sh_css_xnr_config(xnr_config->threshold);
	*/

	if (arg->gc_config) {
		if (copy_from_user(&isp->params.gc_config, arg->gc_config,
			sizeof(*arg->gc_config)))
			return -EFAULT;
		sh_css_set_gc_config(&isp->params.gc_config);
	}

	if (arg->a3a_config) {
		if (copy_from_user(&isp->params.s3a_config, arg->a3a_config,
			sizeof(*arg->a3a_config)))
			return -EFAULT;
		sh_css_set_3a_config(&isp->params.s3a_config);
	}

	return 0;
}

static int __atomisp_set_lsc_table(struct atomisp_device *isp,
			struct atomisp_shading_table *user_st)
{
	unsigned int i;
	unsigned int len_table;
	struct sh_css_shading_table *shading_table;
	struct sh_css_shading_table *old_shading_table;

	if (!user_st)
		return 0;

	old_shading_table = isp->inputs[isp->input_curr].shading_table;

	/* user config is to disable the shading table. */
	if (!user_st->enable) {
		shading_table = NULL;
		goto set_lsc;
	}

	/* Setting a new table. Validate first - all tables must be set */
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		if (!user_st->data[i])
			return -EINVAL;
	}

	/* Shading table size per color */
	if (user_st->width > SH_CSS_MAX_SCTBL_WIDTH_PER_COLOR ||
		user_st->height > SH_CSS_MAX_SCTBL_HEIGHT_PER_COLOR)
		return -EINVAL;

	shading_table = sh_css_shading_table_alloc(user_st->width,
			user_st->height);
	if (!shading_table)
			return -ENOMEM;

	len_table = user_st->width * user_st->height * ATOMISP_SC_TYPE_SIZE;
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		if (copy_from_user(shading_table->data[i],
			user_st->data[i], len_table)) {
			sh_css_shading_table_free(shading_table);
			return -EFAULT;
		}

	}
	shading_table->sensor_width = user_st->sensor_width;
	shading_table->sensor_height = user_st->sensor_height;
	shading_table->fraction_bits = user_st->fraction_bits;

set_lsc:
	/* set LSC to CSS */
	isp->inputs[isp->input_curr].shading_table = shading_table;
	sh_css_set_shading_table(shading_table);
	isp->params.sc_en = shading_table != NULL;

	if (old_shading_table)
		sh_css_shading_table_free(old_shading_table);

	return 0;
}

static int __atomisp_set_morph_table(struct atomisp_device *isp,
				struct atomisp_morph_table *user_morph_table)
{
	int ret = -EFAULT;
	unsigned int i;
	struct sh_css_morph_table *morph_table;
	struct sh_css_morph_table *old_morph_table;

	if (!user_morph_table)
		return 0;

	old_morph_table = isp->inputs[isp->input_curr].morph_table;

	morph_table = sh_css_morph_table_allocate(user_morph_table->width,
				user_morph_table->height);
	if (!morph_table)
		return -ENOMEM;

	for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
		if (copy_from_user(morph_table->coordinates_x[i],
			user_morph_table->coordinates_x[i],
			user_morph_table->height * user_morph_table->width *
			sizeof(*user_morph_table->coordinates_x[i])))
			goto error;

		if (copy_from_user(morph_table->coordinates_y[i],
			user_morph_table->coordinates_y[i],
			user_morph_table->height * user_morph_table->width *
			sizeof(*user_morph_table->coordinates_y[i])))
			goto error;
	}

	isp->inputs[isp->input_curr].morph_table = morph_table;
	if (isp->params.gdc_cac_en)
		sh_css_set_morph_table(morph_table);

	if (old_morph_table)
		sh_css_morph_table_free(old_morph_table);

	return 0;

error:
	if (morph_table)
		sh_css_morph_table_free(morph_table);
	return ret;
}

/*
* Function to configure ISP parameters
*/
int atomisp_set_parameters(struct atomisp_device *isp,
			struct atomisp_parameters *arg)
{
	int ret;

	ret = __atomisp_set_general_isp_parameters(isp, arg);
	if (ret)
		return ret;

	ret = __atomisp_set_lsc_table(isp, arg->shading_table);
	if (ret)
		return ret;

	ret = __atomisp_set_morph_table(isp, arg->morph_table);
	if (ret)
		return ret;

	/* indicate to CSS that we have parametes to be updated */
	isp->params.css_update_params_needed = true;

	return 0;
}

/*
 * Function to set/get isp parameters to isp
 */
int atomisp_param(struct atomisp_device *isp, int flag,
		  struct atomisp_parm *config)
{
	/* Read parameter for 3A binary info */
	if (flag == 0) {
		if (&config->info == NULL) {
			dev_err(isp->dev, "ERROR: NULL pointer in grid_info\n");
			return -EINVAL;
		}
		atomisp_curr_user_grid_info(isp, &config->info);
		return 0;
	}

	memcpy(&isp->params.wb_config, &config->wb_config,
	       sizeof(struct sh_css_wb_config));
	memcpy(&isp->params.ob_config, &config->ob_config,
	       sizeof(struct sh_css_ob_config));
	memcpy(&isp->params.dp_config, &config->dp_config,
	       sizeof(struct sh_css_dp_config));
	memcpy(&isp->params.de_config, &config->de_config,
	       sizeof(struct sh_css_de_config));
	memcpy(&isp->params.ce_config, &config->ce_config,
	       sizeof(struct sh_css_ce_config));
	memcpy(&isp->params.nr_config, &config->nr_config,
	       sizeof(struct sh_css_nr_config));
	memcpy(&isp->params.ee_config, &config->ee_config,
	       sizeof(struct sh_css_ee_config));
	memcpy(&isp->params.tnr_config, &config->tnr_config,
	       sizeof(struct sh_css_tnr_config));

	if (isp->params.color_effect == V4L2_COLORFX_NEGATIVE) {
		config->cc_config.matrix[3] = -config->cc_config.matrix[3];
		config->cc_config.matrix[4] = -config->cc_config.matrix[4];
		config->cc_config.matrix[5] = -config->cc_config.matrix[5];
		config->cc_config.matrix[6] = -config->cc_config.matrix[6];
		config->cc_config.matrix[7] = -config->cc_config.matrix[7];
		config->cc_config.matrix[8] = -config->cc_config.matrix[8];
	}

	if (isp->params.color_effect != V4L2_COLORFX_SEPIA &&
	    isp->params.color_effect != V4L2_COLORFX_BW) {
		memcpy(&isp->params.cc_config, &config->cc_config,
		       sizeof(struct sh_css_cc_config));
		sh_css_set_cc_config(&isp->params.cc_config);
	}

	sh_css_set_wb_config(&isp->params.wb_config);
	sh_css_set_ob_config(&isp->params.ob_config);
	sh_css_set_de_config(&isp->params.de_config);
	sh_css_set_ce_config(&isp->params.ce_config);
	sh_css_set_dp_config(&isp->params.dp_config);
	sh_css_set_nr_config(&isp->params.nr_config);
	sh_css_set_ee_config(&isp->params.ee_config);
	sh_css_set_tnr_config(&isp->params.tnr_config);
	isp->params.css_update_params_needed = true;

	return 0;
}

/*
 * Function to configure color effect of the image
 */
int atomisp_color_effect(struct atomisp_device *isp, int flag, __s32 *effect)
{
	const struct sh_css_cc_config *cc_config;
	const struct sh_css_macc_table *macc_table;
	const struct sh_css_ctc_table *ctc_table;
	int ret = 0;
	struct v4l2_control control;

	if (flag == 0) {
		*effect = isp->params.color_effect;
		return 0;
	}


	control.id = V4L2_CID_COLORFX;
	control.value = *effect;
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				core, s_ctrl, &control);
	/*
	 * if set color effect to sensor successfully, return
	 * 0 directly.
	 */
	if (!ret) {
		isp->params.color_effect = (u32)*effect;
		return 0;
	}

	if (*effect == isp->params.color_effect)
		return 0;

	/*
	 * restore the default cc and ctc table config:
	 * when change from sepia/mono to macc effect, the default
	 * cc and ctc table should be used.
	 */
	cc_config = isp->params.default_cc_config;
	ctc_table = isp->params.default_ctc_table;

	/*
	 * set macc enable to false by default:
	 * when change from macc to sepia/mono,
	 * isp->params.macc_en should be set to false.
	 */
	isp->params.macc_en = false;
	macc_table = isp->params.default_macc_table;

	switch (*effect) {
	case V4L2_COLORFX_NONE:
		macc_table = &isp->params.macc_table;
		isp->params.macc_en = true;
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
	isp->params.css_update_params_needed = true;
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
	isp->params.bad_pixel_en = !!*value;

	return 0;
}

/*
 * Function to configure bad pixel correction params
 */
int atomisp_bad_pixel_param(struct atomisp_device *isp, int flag,
			    struct atomisp_dp_config *config)
{
	if (flag == 0) {
		/* Get bad pixel from current setup */
		memcpy(config, &isp->params.dp_config, sizeof(*config));
	} else {
		/* Set bad pixel to isp parameters */
		memcpy(&isp->params.dp_config, config, sizeof(*config));
		sh_css_set_dp_config(&isp->params.dp_config);
		isp->params.css_update_params_needed = true;
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
		isp->params.video_dis_en = !!*value;

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

static unsigned int
atomisp_bytesperline_to_padded_width(unsigned int bytesperline,
				     enum sh_css_frame_format format)
{
	switch (format) {
	case SH_CSS_FRAME_FORMAT_UYVY:
	case SH_CSS_FRAME_FORMAT_YUYV:
	case SH_CSS_FRAME_FORMAT_RAW:
	case SH_CSS_FRAME_FORMAT_RGB565:
		return bytesperline/2;
	case SH_CSS_FRAME_FORMAT_RGBA888:
		return bytesperline/4;
	/* The following cases could be removed, but we leave them
	   in to document the formats that are included. */
	case SH_CSS_FRAME_FORMAT_NV11:
	case SH_CSS_FRAME_FORMAT_NV12:
	case SH_CSS_FRAME_FORMAT_NV16:
	case SH_CSS_FRAME_FORMAT_NV21:
	case SH_CSS_FRAME_FORMAT_NV61:
	case SH_CSS_FRAME_FORMAT_YV12:
	case SH_CSS_FRAME_FORMAT_YV16:
	case SH_CSS_FRAME_FORMAT_YUV420:
	case SH_CSS_FRAME_FORMAT_YUV420_16:
	case SH_CSS_FRAME_FORMAT_YUV422:
	case SH_CSS_FRAME_FORMAT_YUV422_16:
	case SH_CSS_FRAME_FORMAT_YUV444:
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
	case SH_CSS_FRAME_FORMAT_PLANAR_RGB888:
	case SH_CSS_FRAME_FORMAT_QPLANE6:
	case SH_CSS_FRAME_FORMAT_BINARY_8:
	default:
		return bytesperline;
	}
}

static int
atomisp_v4l2_framebuffer_to_sh_css_frame(const struct v4l2_framebuffer *arg,
					 struct sh_css_frame **result)
{
	struct sh_css_frame *res;
	unsigned int padded_width;
	enum sh_css_frame_format sh_format;
	char *tmp_buf = NULL;
	int ret = 0;

	sh_format = v4l2_fmt_to_sh_fmt(arg->fmt.pixelformat);
	padded_width = atomisp_bytesperline_to_padded_width(
					arg->fmt.bytesperline, sh_format);

	/* Note: the padded width on an sh_css_frame is in elements, not in
	   bytes. The RAW frame we use here should always be a 16bit RAW
	   frame. This is why we bytesperline/2 is equal to the padded with */
	if (sh_css_frame_allocate(&res, arg->fmt.width, arg->fmt.height,
				  sh_format, padded_width, 0)) {
		ret = -ENOMEM;
		goto err;
	}

	tmp_buf = vmalloc(arg->fmt.sizeimage);
	if (!tmp_buf) {
		ret = -ENOMEM;
		goto err;
	}
	if (copy_from_user(tmp_buf, (void __user __force *)arg->base,
			   arg->fmt.sizeimage)) {
		ret = -EFAULT;
		goto err;
	}

	if (hmm_store((void *)res->data, tmp_buf, arg->fmt.sizeimage)) {
		ret = -EINVAL;
		goto err;
	}

err:
	if (ret && res)
		sh_css_frame_free(res);
	if (tmp_buf)
		vfree(tmp_buf);
	if (ret == 0)
		*result = res;
	return ret;
}

/*
 * Function to configure fixed pattern noise table
 */
int atomisp_fixed_pattern_table(struct atomisp_device *isp,
				struct v4l2_framebuffer *arg)
{
	struct sh_css_frame *raw_black_frame = NULL;
	int ret;

	if (arg == NULL)
		return -EINVAL;

	ret = atomisp_v4l2_framebuffer_to_sh_css_frame(arg, &raw_black_frame);
	if (ret)
		return ret;
	if (sh_css_set_black_frame(raw_black_frame) != sh_css_success)
		ret = -ENOMEM;

	sh_css_frame_free(raw_black_frame);
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
	isp->params.css_update_params_needed = true;
	isp->params.false_color = *value;
	return 0;
}

/*
 * Function to configure bad pixel correction params
 */
int atomisp_false_color_param(struct atomisp_device *isp, int flag,
			      struct atomisp_de_config *config)
{
	if (flag == 0) {
		/* Get false color from current setup */
		memcpy(config, &isp->params.de_config, sizeof(*config));
	} else {
		/* Set false color to isp parameters */
		memcpy(&isp->params.de_config, config, sizeof(*config));
		sh_css_set_de_config(&isp->params.de_config);
		isp->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to configure white balance params
 */
int atomisp_white_balance_param(struct atomisp_device *isp, int flag,
	struct atomisp_wb_config *config)
{
	if (flag == 0) {
		/* Get white balance from current setup */
		memcpy(config, &isp->params.wb_config, sizeof(*config));
	} else {
		/* Set white balance to isp parameters */
		memcpy(&isp->params.wb_config, config, sizeof(*config));
		sh_css_set_wb_config(&isp->params.wb_config);
		isp->params.css_update_params_needed = true;
	}

	return 0;
}

int atomisp_3a_config_param(struct atomisp_device *isp, int flag,
			    struct atomisp_3a_config *config)
{
	dev_dbg(isp->dev, ">%s %d\n", __func__, flag);

	if (flag == 0) {
		/* Get white balance from current setup */
		memcpy(config, &isp->params.s3a_config, sizeof(*config));
	} else {
		/* Set white balance to isp parameters */
		memcpy(&isp->params.s3a_config, config, sizeof(*config));
		sh_css_set_3a_config(&isp->params.s3a_config);
		isp->params.css_update_params_needed = true;
		/* isp->params.s3a_buf_data_valid = false; */
	}

	dev_dbg(isp->dev, "<%s %d\n", __func__, flag);
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

	if (*value == 0)
		sh_css_set_shading_table(NULL);
	else
		sh_css_set_shading_table(
			isp->inputs[isp->input_curr].shading_table);

	isp->params.sc_en = *value;

	return 0;
}

/*
 * Function to setup digital zoom
 */
int atomisp_digital_zoom(struct atomisp_device *isp, int flag, __s32 *value)
{
	u32 zoom;
	unsigned int max_zoom =
		IS_ISP2400(isp) ? MRFLD_MAX_ZOOM_FACTOR : MFLD_MAX_ZOOM_FACTOR;

	if (flag == 0) {
		sh_css_get_zoom_factor(&zoom, &zoom);
		*value = max_zoom - zoom;
	} else {
		if (*value < 0)
			return -EINVAL;

		zoom = max_zoom - min_t(u32, max_zoom - 1, *value);

		dev_dbg(isp->dev, "%s, zoom: %d\n", __func__, zoom);
		sh_css_set_zoom_factor(zoom, zoom);
		isp->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to get sensor specific info for current resolution,
 * which will be used for auto exposure conversion.
 */
int atomisp_get_sensor_mode_data(struct atomisp_device *isp,
				 struct atomisp_sensor_mode_data *config)
{
	struct camera_mipi_info *mipi_info;

	mipi_info = atomisp_to_sensor_mipi_info(
		isp->inputs[isp->input_curr].camera);
	if (mipi_info == NULL)
		return -EINVAL;

	memcpy(config, &mipi_info->data, sizeof(*config));
	return 0;
}

int atomisp_get_fmt(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_err(isp->dev, "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix = pipe->pix;

	return 0;
}


/* This function looks up the closest available resolution. */
int atomisp_try_fmt(struct video_device *vdev, struct v4l2_format *f,
						bool *res_overflow)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct v4l2_mbus_framefmt snr_mbus_fmt;
	const struct atomisp_format_bridge *fmt;
	int ret;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_err(isp->dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	if (isp->inputs[isp->input_curr].camera == NULL)
		return -EINVAL;

	fmt = atomisp_get_format_bridge(f->fmt.pix.pixelformat);
	if (fmt == NULL) {
		dev_err(isp->dev, "unsupported pixelformat!\n");
		fmt = atomisp_output_fmts;
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
		return 0;
#endif
	snr_mbus_fmt.code = fmt->mbus_code;
	snr_mbus_fmt.width = f->fmt.pix.width;
	snr_mbus_fmt.height = f->fmt.pix.height;

	dev_dbg(isp->dev, "try_mbus_fmt: asking for %ux%u\n",
		snr_mbus_fmt.width, snr_mbus_fmt.height);

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			video, try_mbus_fmt, &snr_mbus_fmt);
	if (ret)
		return ret;

	dev_dbg(isp->dev, "try_mbus_fmt: got %ux%u\n",
		snr_mbus_fmt.width, snr_mbus_fmt.height);

	fmt = atomisp_get_format_bridge_from_mbus(snr_mbus_fmt.code);
	if (fmt == NULL) {
		dev_err(isp->dev, "unknown sensor format 0x%8.8x\n",
			snr_mbus_fmt.code);
		return -EINVAL;
	}

	f->fmt.pix.pixelformat = fmt->pixelformat;

	if (snr_mbus_fmt.width < f->fmt.pix.width
	    && snr_mbus_fmt.height < f->fmt.pix.height) {
		f->fmt.pix.width = snr_mbus_fmt.width;
		f->fmt.pix.height = snr_mbus_fmt.width;
		/* Set the flag when resolution requested is
		 * beyond the max value supported by sensor
		 */
		if (res_overflow != NULL)
			*res_overflow = true;
	}

	/* app vs isp */
	f->fmt.pix.width = rounddown(
		clamp_t(u32, f->fmt.pix.width, ATOM_ISP_MIN_WIDTH,
			ATOM_ISP_MAX_WIDTH), ATOM_ISP_STEP_WIDTH);
	f->fmt.pix.height = rounddown(
		clamp_t(u32, f->fmt.pix.height, ATOM_ISP_MIN_HEIGHT,
			ATOM_ISP_MAX_HEIGHT), ATOM_ISP_STEP_HEIGHT);

	return 0;
}

static int
atomisp_try_fmt_file(struct atomisp_device *isp, struct v4l2_format *f)
{
	u32 width = f->fmt.pix.width;
	u32 height = f->fmt.pix.height;
	u32 pixelformat = f->fmt.pix.pixelformat;
	enum v4l2_field field = f->fmt.pix.field;
	u32 depth;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		dev_err(isp->dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	if (!atomisp_get_format_bridge(pixelformat)) {
		dev_err(isp->dev, "Wrong output pixelformat\n");
		return -EINVAL;
	}

	depth = get_pixel_depth(pixelformat);

	if (!field || field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (field != V4L2_FIELD_NONE) {
		dev_err(isp->dev, "Wrong output field\n");
		return -EINVAL;
	}

	f->fmt.pix.field = field;
	f->fmt.pix.width = clamp_t(u32,
				   rounddown(width, (u32)ATOM_ISP_STEP_WIDTH),
				   ATOM_ISP_MIN_WIDTH, ATOM_ISP_MAX_WIDTH);
	f->fmt.pix.height = clamp_t(u32, rounddown(height,
						   (u32)ATOM_ISP_STEP_HEIGHT),
				    ATOM_ISP_MIN_HEIGHT, ATOM_ISP_MAX_HEIGHT);
	f->fmt.pix.bytesperline = (width * depth) >> 3;

	return 0;
}

static mipi_port_ID_t __get_mipi_port(struct atomisp_device *isp,
				      enum atomisp_camera_port port)
{
	switch (port) {
	case ATOMISP_CAMERA_PORT_PRIMARY:
		return MIPI_PORT0_ID;
	case ATOMISP_CAMERA_PORT_SECONDARY:
		return MIPI_PORT1_ID;
	case ATOMISP_CAMERA_PORT_THIRD:
		if (MIPI_PORT1_ID + 1 != N_MIPI_PORT_ID)
			return MIPI_PORT1_ID + 1;
		/* go through down for else case */
	default:
		dev_err(isp->dev, "unsupported port: %d\n", port);
		return MIPI_PORT0_ID;
	}
}

static inline void atomisp_set_sensor_mipi_to_isp(struct atomisp_device *isp,
		struct camera_mipi_info *mipi_info)
{
	/* Compatibility for sensors which provide no media bus code
	 * in s_mbus_framefmt() nor support pad formats. */
	if (mipi_info->input_format != -1) {
		atomisp_css_input_set_bayer_order(isp,
						mipi_info->raw_bayer_order);
		atomisp_css_input_set_format(isp, mipi_info->input_format);
	}
	sh_css_input_configure_port(__get_mipi_port(isp, mipi_info->port),
				    mipi_info->num_lanes, 0xffff4);
}

static int __enable_continuous_mode(struct atomisp_device *isp, bool enable)
{
	dev_dbg(isp->dev, "continuous mode %d, raw buffers %d, stop preview %d\n",
		enable, isp->isp_subdev.continuous_raw_buffer_size->val,
		!isp->isp_subdev.continuous_viewfinder->val);
	atomisp_css_capture_set_mode(isp, CSS_CAPTURE_MODE_PRIMARY);
	atomisp_css_capture_enable_online(isp, !enable);
	atomisp_css_preview_enable_online(isp, !enable);
	atomisp_css_enable_continuous(isp, enable);
	atomisp_css_enable_cont_capt(isp, enable,
				!isp->isp_subdev.continuous_viewfinder->val);

	/*
	 * WORKROUND: To be removed when NUM_CONTINUOUS_FRAMES set to 10
	 * for ISP2400A0
	 */
	if ((intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		&& (isp->isp_subdev.continuous_raw_buffer_size->val > 5))
		isp->isp_subdev.continuous_raw_buffer_size->val = 5;
	if (sh_css_continuous_set_num_raw_frames(
			isp->isp_subdev.continuous_raw_buffer_size->val)
		!= sh_css_success) {
		dev_err(isp->dev, "css_continuous_set_num_raw_frames failed\n");
		return -EINVAL;
	}

	if (!enable) {
		atomisp_css_enable_raw_binning(isp, false);
		atomisp_css_input_set_two_pixels_per_clock(isp, false);
	}

	if (isp->inputs[isp->input_curr].type != TEST_PATTERN &&
		isp->inputs[isp->input_curr].type != FILE_INPUT)
		atomisp_css_input_set_mode(isp, CSS_INPUT_MODE_SENSOR);

	return atomisp_update_run_mode(isp);
}

static enum sh_css_err configure_pp_input_nop(unsigned int width,
					      unsigned int height)
{
	return 0;
}

static int atomisp_set_fmt_to_isp(struct video_device *vdev,
				   struct sh_css_frame_info *output_info,
				   struct sh_css_frame_info *raw_output_info,
				   int width, int height,
				  unsigned int pixelformat,
				  unsigned int source_pad)
{
	struct camera_mipi_info *mipi_info;
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *asd = &isp->isp_subdev;
	const struct atomisp_format_bridge *format;
	struct v4l2_rect *isp_sink_crop;
	enum sh_css_err (*configure_output)(unsigned int width,
					    unsigned int height,
					    enum sh_css_frame_format sh_fmt);
	enum sh_css_err (*get_frame_info)(struct sh_css_frame_info *finfo);
	enum sh_css_err (*configure_pp_input)(unsigned int width,
					      unsigned int height) =
		configure_pp_input_nop;
	int ret;

	isp_sink_crop = atomisp_subdev_get_rect(
		&isp->isp_subdev.subdev, NULL, V4L2_SUBDEV_FORMAT_ACTIVE,
		ATOMISP_SUBDEV_PAD_SINK, V4L2_SEL_TGT_CROP);

	format = atomisp_get_format_bridge(pixelformat);
	if (format == NULL)
		return -EINVAL;

	if (isp->inputs[isp->input_curr].type != TEST_PATTERN &&
		isp->inputs[isp->input_curr].type != FILE_INPUT) {
		mipi_info = atomisp_to_sensor_mipi_info(
			isp->inputs[isp->input_curr].camera);
		if (!mipi_info) {
			dev_err(isp->dev, "mipi_info is NULL\n");
			return -EINVAL;
		}
		atomisp_set_sensor_mipi_to_isp(isp, mipi_info);

		if ((format->sh_fmt == SH_CSS_FRAME_FORMAT_RAW) &&
		     raw_output_format_match_input(
			mipi_info->input_format, pixelformat))
			return -EINVAL;
	}

	/*
	 * Configure viewfinder also if enable_vfpp is disabled: the
	 * CSS still requires viewfinder configuration.
	 */
	if (isp->isp_subdev.fmt_auto->val
	    || !isp->isp_subdev.enable_vfpp->val) {
		struct v4l2_rect vf_size = {0};
		struct v4l2_mbus_framefmt vf_ffmt = {0};

		if (width < 640 || height < 480) {
			vf_size.width = width;
			vf_size.height = height;
		} else {
			vf_size.width = 640;
			vf_size.height = 480;
		}

		/* FIXME: proper format name for this one. See
		   atomisp_output_fmts[] in atomisp_v4l2.c */
		vf_ffmt.code = 0x8001;

		atomisp_subdev_set_selection(&asd->subdev, NULL,
					     V4L2_SUBDEV_FORMAT_ACTIVE,
					     ATOMISP_SUBDEV_PAD_SOURCE_VF,
					     V4L2_SEL_TGT_COMPOSE, 0, &vf_size);
		atomisp_subdev_set_ffmt(&asd->subdev, NULL,
					V4L2_SUBDEV_FORMAT_ACTIVE,
					ATOMISP_SUBDEV_PAD_SOURCE_VF, &vf_ffmt);

		isp->isp_subdev.video_out_vf.sh_fmt = SH_CSS_FRAME_FORMAT_YUV420;

		if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO
		    || !isp->isp_subdev.enable_vfpp->val)
			sh_css_video_configure_viewfinder(
				vf_size.width, vf_size.height,
				isp->isp_subdev.video_out_vf.sh_fmt);
		else if (source_pad != ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW)
			sh_css_capture_configure_viewfinder(
				vf_size.width, vf_size.height,
				isp->isp_subdev.video_out_vf.sh_fmt);
	}

	if (isp->isp_subdev.continuous_mode->val) {
		if (isp->isp_subdev.run_mode->val != ATOMISP_RUN_MODE_VIDEO) {
			ret = __enable_continuous_mode(isp, true);

			/*
			 * Enable only if resolution is >= 3M for ISP2400
			 */
			if (IS_ISP2400(isp) && (width >= 2048
						|| height >= 1536)) {
				atomisp_css_enable_raw_binning(isp, true);
				atomisp_css_input_set_two_pixels_per_clock(isp,
									false);
			}

			if (!IS_ISP2400(isp)) {
				/* enable raw binning for output >= 5M */
				if (width >= 2576 || height >= 1936)
					atomisp_css_enable_raw_binning(isp,
									true);
				/* enable 2ppc for CTP if output > 8M */
				if (width > 3264 || height > 2448)
					atomisp_css_input_set_two_pixels_per_clock(
								isp, true);
			}
		} else {
			ret = __enable_continuous_mode(isp, false);
		}
		if (ret)
			return -EINVAL;
	}

	sh_css_disable_vf_pp(!isp->isp_subdev.enable_vfpp->val);

	/* video same in continuouscapture and online modes */
	if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO
	    || !isp->isp_subdev.enable_vfpp->val) {
		configure_output = sh_css_video_configure_output;
		get_frame_info = sh_css_video_get_output_frame_info;
	} else if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW) {
		configure_output = sh_css_preview_configure_output;
		get_frame_info = sh_css_preview_get_output_frame_info;
		configure_pp_input = sh_css_preview_configure_pp_input;
	} else {
		if (format->sh_fmt == SH_CSS_FRAME_FORMAT_RAW) {
			atomisp_css_capture_set_mode(isp, CSS_CAPTURE_MODE_RAW);
		}
		if (!isp->isp_subdev.continuous_mode->val)
			atomisp_css_capture_enable_online(isp,
					isp->params.online_process);

		configure_output = sh_css_capture_configure_output;
		get_frame_info = sh_css_capture_get_output_frame_info;
		configure_pp_input = sh_css_capture_configure_pp_input;

		if (!isp->params.online_process &&
		    !isp->isp_subdev.continuous_mode->val)
			if (sh_css_capture_get_output_raw_frame_info(
						raw_output_info))
				return -EINVAL;
		if (!isp->isp_subdev.continuous_mode->val &&
		    isp->isp_subdev.run_mode->val
		    != ATOMISP_RUN_MODE_STILL_CAPTURE) {
			dev_err(isp->dev,
				    "Need to set the running mode first\n");
			isp->isp_subdev.run_mode->val =
				ATOMISP_RUN_MODE_STILL_CAPTURE;
		}
	}

	ret = configure_output(width, height, format->sh_fmt);
	if (ret) {
		dev_err(isp->dev, "configure_output %ux%u, format %8.8x\n",
			width, height, format->sh_fmt);
		return -EINVAL;
	}
	if (isp->isp_subdev.continuous_mode->val) {
		configure_pp_input(0, 0);
	} else {
		ret = configure_pp_input(isp_sink_crop->width,
					 isp_sink_crop->height);
		if (ret) {
			dev_err(isp->dev, "configure_pp_input %ux%u\n",
				isp_sink_crop->width, isp_sink_crop->height);
			return -EINVAL;
		}
	}
	ret = get_frame_info(output_info);
	if (ret) {
		dev_err(isp->dev, "get_frame_info %ux%u\n", width, height);
		return -EINVAL;
	}

	atomisp_update_grid_info(isp);

	/* Free the raw_dump buffer first */
	sh_css_frame_free(isp->raw_output_frame);
	isp->raw_output_frame = NULL;

	if (!isp->isp_subdev.continuous_mode->val &&
		!isp->params.online_process && !isp->sw_contex.file_input &&
	    sh_css_frame_allocate_from_info(&isp->raw_output_frame,
					       raw_output_info))
		return -ENOMEM;

	return 0;
}

static void atomisp_get_dis_envelop(struct atomisp_device *isp,
			    unsigned int width, unsigned int height,
			    unsigned int *dvs_env_w,
			    unsigned int *dvs_env_h)
{
	/* if subdev type is SOC camera,we do not need to set DVS */
	if (isp->inputs[isp->input_curr].type == SOC_CAMERA)
		isp->params.video_dis_en = 0;

	if (isp->params.video_dis_en &&
	    isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		/* envelope is 20% of the output resolution */
		/*
		 * dvs envelope cannot be round up.
		 * it would cause ISP timeout and color switch issue
		 */
		*dvs_env_w = rounddown(width / 5, ATOM_ISP_STEP_WIDTH);
		*dvs_env_h = rounddown(height / 5, ATOM_ISP_STEP_HEIGHT);
	}

	isp->params.dis_proj_data_valid = false;
	isp->params.css_update_params_needed = true;
}

static int atomisp_set_fmt_to_snr(struct atomisp_device *isp,
			  struct v4l2_format *f, unsigned int pixelformat,
			  unsigned int padding_w, unsigned int padding_h,
			  unsigned int dvs_env_w, unsigned int dvs_env_h)
{
	const struct atomisp_format_bridge *format;
	struct v4l2_mbus_framefmt ffmt;
	int ret;

	format = atomisp_get_format_bridge(pixelformat);
	if (format == NULL)
		return -EINVAL;

	v4l2_fill_mbus_format(&ffmt, &f->fmt.pix, format->mbus_code);
	ffmt.height += padding_h + dvs_env_h;
	ffmt.width += padding_w + dvs_env_w;

	dev_dbg(isp->dev, "s_mbus_fmt: ask %ux%u (padding %ux%u, dvs %ux%u)\n",
		ffmt.width, ffmt.height, padding_w, padding_h,
		dvs_env_w, dvs_env_h);

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera, video,
			       s_mbus_fmt, &ffmt);
	if (ret)
		return ret;

	dev_dbg(isp->dev, "sensor width: %d, height: %d\n",
		ffmt.width, ffmt.height);

	if (ffmt.width < ATOM_ISP_STEP_WIDTH ||
	    ffmt.height < ATOM_ISP_STEP_HEIGHT)
			return -EINVAL;

	return atomisp_subdev_set_ffmt(&isp->isp_subdev.subdev, NULL,
				       V4L2_SUBDEV_FORMAT_ACTIVE,
				       ATOMISP_SUBDEV_PAD_SINK, &ffmt);
}

int atomisp_set_fmt(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	const struct atomisp_format_bridge *format_bridge;
	struct sh_css_frame_info output_info, raw_output_info;
	struct v4l2_format snr_fmt = *f;
	unsigned int dvs_env_w = 0,
		     dvs_env_h = 0;
	unsigned int padding_w = pad_w,
		     padding_h = pad_h;
	bool res_overflow = false;
	struct v4l2_mbus_framefmt isp_sink_fmt;
	struct v4l2_mbus_framefmt isp_source_fmt = {0};
	struct v4l2_rect isp_sink_crop;
	uint16_t source_pad = atomisp_subdev_source_pad(vdev);
	int ret;

	dev_dbg(isp->dev, "setting resolution %ux%u on pad %u\n",
		f->fmt.pix.width, f->fmt.pix.height, source_pad);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    f->type != V4L2_BUF_TYPE_PRIVATE) {
		dev_err(isp->dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	format_bridge = atomisp_get_format_bridge(f->fmt.pix.pixelformat);
	if (format_bridge == NULL)
		return -EINVAL;

	pipe->sh_fmt = format_bridge->sh_fmt;
	pipe->pix.pixelformat = f->fmt.pix.pixelformat;

	if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_VF
	    || (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW
		&& isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO)) {
		if (isp->isp_subdev.fmt_auto->val) {
			struct v4l2_rect *capture_comp =
				atomisp_subdev_get_rect(
					&isp->isp_subdev.subdev, NULL,
					V4L2_SUBDEV_FORMAT_ACTIVE,
					ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE,
					V4L2_SEL_TGT_COMPOSE);
			struct v4l2_rect r = {0};

			r.width = f->fmt.pix.width;
			r.height = f->fmt.pix.height;

			if (capture_comp->width < r.width
			    || capture_comp->height < r.height) {
				r.width = capture_comp->width;
				r.height = capture_comp->height;
			}

			atomisp_subdev_set_selection(
				&isp->isp_subdev.subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE, source_pad,
				V4L2_SEL_TGT_COMPOSE, 0, &r);

			f->fmt.pix.width = r.width;
			f->fmt.pix.height = r.height;
		}

		if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
			sh_css_video_configure_viewfinder(
				f->fmt.pix.width, f->fmt.pix.height,
				format_bridge->sh_fmt);
			sh_css_video_get_viewfinder_frame_info(&output_info);
		} else {
			sh_css_capture_configure_viewfinder(
				f->fmt.pix.width, f->fmt.pix.height,
				format_bridge->sh_fmt);
			sh_css_capture_get_viewfinder_frame_info(&output_info);
		}

		goto done;
	}
	/*
	 * Check whether main resolution configured smaller
	 * than snapshot resolution. If so, force main resolution
	 * to be the same as snapshot resolution
	 */
	if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE) {
		struct v4l2_rect *r;

		r = atomisp_subdev_get_rect(
			&isp->isp_subdev.subdev, NULL,
			V4L2_SUBDEV_FORMAT_ACTIVE,
			ATOMISP_SUBDEV_PAD_SOURCE_VF, V4L2_SEL_TGT_COMPOSE);

		if (r->width && r->height
		    && (r->width > f->fmt.pix.width
			|| r->height > f->fmt.pix.height))
			dev_warn(isp->dev,
				 "Main Resolution config smaller then Vf Resolution. Force to be equal with Vf Resolution.");
	}

	/* V4L2_BUF_TYPE_PRIVATE will set offline processing */
	if (f->type == V4L2_BUF_TYPE_PRIVATE)
		isp->params.online_process = 0;
	else
		isp->params.online_process = 1;

	/* Pipeline configuration done through subdevs. Bail out now. */
	if (!isp->isp_subdev.fmt_auto->val)
		goto set_fmt_to_isp;

	/* get sensor resolution and format */
	ret = atomisp_try_fmt(vdev, &snr_fmt, &res_overflow);
	if (ret)
		return ret;
	f->fmt.pix.width = snr_fmt.fmt.pix.width;
	f->fmt.pix.height = snr_fmt.fmt.pix.height;

	atomisp_subdev_get_ffmt(&isp->isp_subdev.subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK)->code =
		atomisp_get_format_bridge(
			snr_fmt.fmt.pix.pixelformat)->mbus_code;

	isp_sink_fmt = *atomisp_subdev_get_ffmt(&isp->isp_subdev.subdev, NULL,
					    V4L2_SUBDEV_FORMAT_ACTIVE,
					    ATOMISP_SUBDEV_PAD_SINK);

	isp_source_fmt.code = atomisp_get_format_bridge(
		f->fmt.pix.pixelformat)->mbus_code;
	atomisp_subdev_set_ffmt(&isp->isp_subdev.subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				source_pad, &isp_source_fmt);

	if (!atomisp_subdev_format_conversion(isp, source_pad))
		padding_w = 0, padding_h = 0;

	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		padding_w = 12;
		padding_h = 12;
	}

	/* construct resolution supported by isp */
	if (res_overflow && !isp->isp_subdev.continuous_mode->val) {
		f->fmt.pix.width = rounddown(
			clamp_t(u32, f->fmt.pix.width - padding_w,
				ATOM_ISP_MIN_WIDTH,
				ATOM_ISP_MAX_WIDTH), ATOM_ISP_STEP_WIDTH);
		f->fmt.pix.height = rounddown(
			clamp_t(u32, f->fmt.pix.height - padding_h,
				ATOM_ISP_MIN_HEIGHT,
				ATOM_ISP_MAX_HEIGHT), ATOM_ISP_STEP_HEIGHT);
	}

	atomisp_get_dis_envelop(isp, f->fmt.pix.width, f->fmt.pix.height,
				&dvs_env_w, &dvs_env_h);

	/* Only main stream pipe will be here */
	isp->isp_subdev.capture_pad = source_pad;

	/*
	 * set format info to sensor
	 * In continuous mode, resolution is set only if it is higher than
	 * existing value. This because preview pipe will be configured after
	 * capture pipe and usually has lower resolution than capture pipe.
	 */
	if (!isp->isp_subdev.continuous_mode->val ||
	    isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO ||
	    (isp_sink_fmt.width < (f->fmt.pix.width + padding_w + dvs_env_w) &&
	     isp_sink_fmt.height < (f->fmt.pix.height + padding_h +
				    dvs_env_h))) {
		ret = atomisp_set_fmt_to_snr(isp, f, f->fmt.pix.pixelformat,
					     padding_w, padding_h,
					     dvs_env_w, dvs_env_h);
		if (ret)
			return -EINVAL;
	}

	isp_sink_crop = *atomisp_subdev_get_rect(&isp->isp_subdev.subdev, NULL,
						 V4L2_SUBDEV_FORMAT_ACTIVE,
						 ATOMISP_SUBDEV_PAD_SINK,
						 V4L2_SEL_TGT_CROP);

	/* Try to enable YUV downscaling if ISP input is 10 % (either
	 * width or height) bigger than the desired result. */
	if (isp_sink_crop.width * 9 / 10 < f->fmt.pix.width
	    || isp_sink_crop.height * 9 / 10 < f->fmt.pix.height
	    || (atomisp_subdev_format_conversion(isp, source_pad)
		&& isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO)) {
		isp_sink_crop.width = f->fmt.pix.width;
		isp_sink_crop.height = f->fmt.pix.height;
		atomisp_subdev_set_selection(&isp->isp_subdev.subdev, NULL,
					     V4L2_SUBDEV_FORMAT_ACTIVE,
					     ATOMISP_SUBDEV_PAD_SINK,
					     V4L2_SEL_TGT_CROP,
					     V4L2_SEL_FLAG_KEEP_CONFIG,
					     &isp_sink_crop);
		atomisp_subdev_set_selection(&isp->isp_subdev.subdev, NULL,
					     V4L2_SUBDEV_FORMAT_ACTIVE,
					     source_pad, V4L2_SEL_TGT_COMPOSE,
					     0, &isp_sink_crop);
	} else {
		struct v4l2_rect main_compose = {0};

		main_compose.width = isp_sink_crop.width - padding_w;
		main_compose.height =
			DIV_ROUND_UP(main_compose.width * f->fmt.pix.height,
				     f->fmt.pix.width);
		if (main_compose.height > isp_sink_crop.height - padding_h) {
			main_compose.height = isp_sink_crop.height - padding_h;
			main_compose.width =
				DIV_ROUND_UP(main_compose.height *
					     f->fmt.pix.width,
					     f->fmt.pix.height);
		}

		atomisp_subdev_set_selection(&isp->isp_subdev.subdev, NULL,
					     V4L2_SUBDEV_FORMAT_ACTIVE,
					     ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE,
					     V4L2_SEL_TGT_COMPOSE, 0,
					     &main_compose);
	}

set_fmt_to_isp:
	ret = atomisp_set_fmt_to_isp(vdev, &output_info, &raw_output_info,
				     f->fmt.pix.width, f->fmt.pix.height,
				     f->fmt.pix.pixelformat, source_pad);
	if (ret)
		return -EINVAL;
done:
	pipe->pix.width = f->fmt.pix.width;
	pipe->pix.height = f->fmt.pix.height;
	pipe->pix.pixelformat = f->fmt.pix.pixelformat;
	pipe->pix.bytesperline =
		DIV_ROUND_UP(format_bridge->depth * output_info.padded_width,
			     8);
	pipe->pix.sizeimage =
	    PAGE_ALIGN(f->fmt.pix.height * pipe->pix.bytesperline);
	if (f->fmt.pix.field == V4L2_FIELD_ANY)
		f->fmt.pix.field = V4L2_FIELD_NONE;
	pipe->pix.field = f->fmt.pix.field;

	f->fmt.pix = pipe->pix;
	f->fmt.pix.priv = PAGE_ALIGN(pipe->pix.width *
				     pipe->pix.height * 2);

	pipe->capq.field = f->fmt.pix.field;

	/*
	 * If in video 480P case, no GFX throttle
	 */
	if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE) {
		if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO &&
		    f->fmt.pix.width == 720 && f->fmt.pix.height == 480)
			isp->need_gfx_throttle = false;
		else
			isp->need_gfx_throttle = true;
	}

	return 0;
}

int atomisp_set_fmt_file(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct v4l2_mbus_framefmt ffmt = {0};
	const struct atomisp_format_bridge *format_bridge;
	int ret;

	dev_dbg(isp->dev, "setting fmt %ux%u 0x%x for file inject\n",
		f->fmt.pix.width, f->fmt.pix.height, f->fmt.pix.pixelformat);
	ret = atomisp_try_fmt_file(isp, f);
	if (ret) {
		dev_err(isp->dev, "atomisp_try_fmt_file err: %d\n", ret);
		return ret;
	}

	format_bridge = atomisp_get_format_bridge(f->fmt.pix.pixelformat);
	if (format_bridge == NULL) {
		dev_dbg(isp->dev, "atomisp_get_format_bridge err! fmt:0x%x\n",
				f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	pipe->pix = f->fmt.pix;
	atomisp_css_input_set_mode(isp, CSS_INPUT_MODE_FIFO);
	sh_css_input_configure_port(
		__get_mipi_port(isp, ATOMISP_CAMERA_PORT_PRIMARY), 2, 0xffff4);

	ffmt.width = f->fmt.pix.width;
	ffmt.height = f->fmt.pix.height;
	ffmt.code = format_bridge->mbus_code;

	atomisp_subdev_set_ffmt(&isp->isp_subdev.subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK, &ffmt);

	return 0;
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
		sh_css_set_shading_table(NULL);
		isp->params.sc_en = 0;
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

	free_table = isp->inputs[isp->input_curr].shading_table;
	isp->inputs[isp->input_curr].shading_table = shading_table;
	sh_css_set_shading_table(shading_table);
	isp->params.sc_en = 1;

out:
	if (free_table != NULL)
		sh_css_shading_table_free(free_table);

	return ret;
}

/*Turn off ISP dphy */
int atomisp_ospm_dphy_down(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;
	int timeout = 100;
	bool idle;
	u32 reg;

	dev_dbg(isp->dev, "%s\n", __func__);

	/* if ISP timeout, we can force powerdown */
	if (isp->isp_timeout)
		goto done;

	if (!atomisp_users(isp))
		goto done;

	idle = sh_css_hrt_system_is_idle();
	dev_dbg(isp->dev, "%s system_is_idle:%d\n", __func__, idle);
	while (!idle && timeout--) {
		udelay(20);
		idle = sh_css_hrt_system_is_idle();
	}

	if (timeout < 0) {
		dev_err(isp->dev, "Timeout to stop ISP HW\n");
		/* force power down here */
		/* return -EINVAL; */
	}

done:
	if (IS_ISP2400(isp)) {
		/*
		 * MRFLD IUNIT DPHY is located in an always-power-on island
		 * MRFLD HW design need all CSI ports are disabled before
		 * powering down the IUNIT.
		 */
		pci_read_config_dword(isp->pdev, MRFLD_PCI_CSI_CONTROL, &reg);
		reg |= MRFLD_ALL_CSI_PORTS_OFF_MASK;
		pci_write_config_dword(isp->pdev, MRFLD_PCI_CSI_CONTROL, reg);
	} else {
		/* power down DPHY */
		pwr_cnt = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT,
							MFLD_CSI_CONTROL);
		pwr_cnt |= 0x300;
		intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT,
						MFLD_CSI_CONTROL, pwr_cnt);
	}

	isp->sw_contex.power_state = ATOM_ISP_POWER_DOWN;
	return 0;
}

/*Turn on ISP dphy */
int atomisp_ospm_dphy_up(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;
	dev_dbg(isp->dev, "%s\n", __func__);

	/* MRFLD IUNIT DPHY is located in an always-power-on island */
	if (!IS_ISP2400(isp)) {
		/* power on DPHY */
		pwr_cnt = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT,
							MFLD_CSI_CONTROL);
		pwr_cnt &= ~0x300;
		intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT,
						MFLD_CSI_CONTROL, pwr_cnt);
	}

	isp->sw_contex.power_state = ATOM_ISP_POWER_UP;

	return 0;
}


int atomisp_exif_makernote(struct atomisp_device *isp,
			   struct atomisp_makernote_info *config)
{
	struct v4l2_control ctrl;

	ctrl.id = V4L2_CID_FOCAL_ABSOLUTE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				 core, g_ctrl, &ctrl))
		dev_warn(isp->dev, "failed to g_ctrl for focal length\n");
	else
		config->focal_length = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_ABSOLUTE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				core, g_ctrl, &ctrl))
		dev_warn(isp->dev, "failed to g_ctrl for f-number\n");
	else
		config->f_number_curr = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_RANGE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				core, g_ctrl, &ctrl))
		dev_warn(isp->dev, "failed to g_ctrl for f number range\n");
	else
		config->f_number_range = ctrl.value;

	return 0;
}

int atomisp_offline_capture_configure(struct atomisp_device *isp,
			      struct atomisp_cont_capture_conf *cvf_config)
{
	isp->params.offline_parm = *cvf_config;

	if (isp->params.offline_parm.num_captures) {
		if (isp->streaming == ATOMISP_DEVICE_STREAMING_DISABLED) {
			int num_raw_frames =
				min_t(int, ATOMISP_CONT_RAW_FRAMES,
				      isp->params.offline_parm.num_captures
				      + 3);
			/* WORKROUND: To be removed when NUM_CONTINUOUS_FRAMES set to 10 */
			if ((intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_VALLEYVIEW2
				|| intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
				&& (num_raw_frames > 5))
				num_raw_frames = 5;
			/* TODO: this can be removed once user-space
			 *       has been updated to use control API */
			isp->isp_subdev.continuous_raw_buffer_size->val =
				num_raw_frames;
		}

		isp->isp_subdev.continuous_mode->val = true;
	} else {
		isp->isp_subdev.continuous_mode->val = false;
		__enable_continuous_mode(isp, false);
	}

	return 0;
}

int atomisp_flash_enable(struct atomisp_device *isp, int num_frames)
{
	if (num_frames < 0) {
		dev_dbg(isp->dev, "%s ERROR: num_frames: %d\n", __func__,
				num_frames);
		return -EINVAL;
	}
	/* a requested flash is still in progress. */
	if (num_frames && isp->params.flash_state != ATOMISP_FLASH_IDLE) {
		dev_dbg(isp->dev, "%s flash busy: %d frames left: %d\n",
				__func__, isp->params.flash_state,
				isp->params.num_flash_frames);
		return -EBUSY;
	}

	isp->params.num_flash_frames = num_frames;
	isp->params.flash_state = ATOMISP_FLASH_REQUESTED;
	return 0;
}
