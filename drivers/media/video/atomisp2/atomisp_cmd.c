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
#include "linux/intel_mid_pm.h"
#include <linux/kernel.h>
#include <media/v4l2-event.h>
#include <asm/intel-mid.h>

#define ATOMISP_DEFAULT_DTRACE_LEVEL 5
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

int atomisp_dtrace_level = ATOMISP_DEFAULT_DTRACE_LEVEL;
#define atomisp_dtrace(level, format, args...)          \
	do {                                           \
		if (atomisp_dtrace_level >= level)       \
			trace_printk(format, ## args); \
	} while (0)

/*
 * atomisp_kernel_malloc: chooses whether kmalloc() or vmalloc() is preferable.
 *
 * It is also a wrap functions to pass into css framework.
 */
void *atomisp_kernel_malloc(size_t bytes)
{
	/* vmalloc() is preferable if allocating more than 1 page */
	if (bytes > PAGE_SIZE) {
		void *ptr = vmalloc(bytes);
		if (ptr == NULL)
			return NULL;
		memset(ptr, 0, bytes);
		return ptr;
	}

	return kzalloc(bytes, GFP_KERNEL);
}

/* Free buffer allocated with atomisp_kernel_malloc() helper */
void atomisp_kernel_free(void *ptr)
{
	/* Verify if buffer was allocated by vmalloc() or kmalloc() */
	if (is_vmalloc_addr(ptr))
		vfree(ptr);
	else
		kfree(ptr);
}

static void atomisp_buf_done(struct atomisp_device *isp,
			     int error, enum sh_css_buffer_type buf_type);

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
 * reset and restore ISP
 */
int atomisp_reset(struct atomisp_device *isp)
{
	/* Reset ISP by power-cycling it */
	int ret = 0;
	v4l2_dbg(2, dbg_level, &atomisp_dev, "%s\n",__func__);
	sh_css_suspend();
	ret = pm_runtime_put_sync(isp->dev);
	if (ret) {
		v4l2_err(&atomisp_dev, "can not disable ISP power\n");
	} else {
		ret = pm_runtime_get_sync(isp->dev);
		if (ret)
			v4l2_err(&atomisp_dev, "can not enable ISP power\n");
	}
	sh_css_resume();
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

#ifndef CONFIG_X86_MRFLD
static void atomisp_sof_event(struct atomisp_device *isp)
{
	struct v4l2_event event;

	memset(&event, 0, sizeof(event));
	event.type = V4L2_EVENT_FRAME_SYNC;
	event.u.frame_sync.frame_sequence = atomic_read(&isp->sequence);

	v4l2_event_queue(isp->isp_subdev.subdev.devnode, &event);
}
#endif /* CONFIG_X86_MRFLD */

static void print_csi_rx_errors(void)
{
	u32 infos = 0;
	sh_css_rx_get_interrupt_info(&infos);

	v4l2_err(&atomisp_dev, "CSI Receiver errors:\n");
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
	unsigned int irq_infos = 0;
	int err;

	err = sh_css_translate_interrupt(&irq_infos);
	v4l2_dbg(5, dbg_level, &atomisp_dev, "irq:0x%x\n", irq_infos);

	if (err != sh_css_success) {
		v4l2_warn(&atomisp_dev, "%s:failed to translate irq (err = %d,"
			  " infos = %d)\n", __func__, err, irq_infos);
		return IRQ_NONE;
	}

#ifndef CONFIG_X86_MRFLD
	if (irq_infos & SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF) {
		atomic_inc(&isp->sequence);
		atomisp_sof_event(isp);
	}
#else /* CONFIG_X86_MRFLD */
	if (irq_infos & SH_CSS_IRQ_INFO_PIPELINE_DONE)
		atomic_inc(&isp->sequence);
#endif /* CONFIG_X86_MRFLD */

#ifdef CONFIG_X86_MRFLD
	if ((irq_infos & SH_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR) ||
		(irq_infos & SH_CSS_IRQ_INFO_IF_ERROR)) {
#else
	if (irq_infos & SH_CSS_IRQ_INFO_CSS_RECEIVER_ERROR) {
#endif
		/* handle mipi receiver error */
		u32 rx_infos;
		print_csi_rx_errors();
		sh_css_rx_get_interrupt_info(&rx_infos);
		sh_css_rx_clear_interrupt_info(rx_infos);
		/* TODO: handle SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN */
	}

	if (irq_infos & SH_CSS_IRQ_INFO_INVALID_FIRST_FRAME) {
		isp->sw_contex.invalid_frame = 1;
		isp->sw_contex.invalid_vf_frame = 1;
		isp->sw_contex.invalid_s3a = 1;
		isp->sw_contex.invalid_dis = 1;
	}

	atomic_set(&isp->wdt_count, 0);
	mod_timer(&isp->wdt, jiffies + ATOMISP_ISP_TIMEOUT_DURATION);

	/* Clear irq reg at PENWELL B0 */
	pci_read_config_dword(isp->pdev, PCI_INTERRUPT_CTRL, &msg_ret);
	msg_ret |= 1 << INTR_IIR;
	pci_write_config_dword(isp->pdev, PCI_INTERRUPT_CTRL, msg_ret);

	return isp->sw_contex.isp_streaming ? IRQ_WAKE_THREAD : IRQ_HANDLED;
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
	if (IS_MRFLD)
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

static void atomisp_pipe_reset(struct atomisp_device *isp)
{
	int ret;
	enum sh_css_pipe_id css_pipe_id;

	ret = atomisp_get_css_pipe_id(isp, &css_pipe_id);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
			"get css pipe id fails: %d\n", ret);
		return;
	}

	v4l2_warn(&atomisp_dev, "ISP timeout. Recovering\n");

#ifndef CONFIG_X86_MRFLD
	if (!isp->sw_contex.file_input)
		sh_css_enable_interrupt(SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF,
					false);
#endif /* CONFIG_X86_MRFLD */

	switch (isp->sw_contex.run_mode) {
	case CI_MODE_STILL_CAPTURE:
		sh_css_capture_stop();
		break;
	case CI_MODE_PREVIEW:
		sh_css_preview_stop();
		break;
	case CI_MODE_VIDEO:
		sh_css_video_stop();
		break;
	}

	/* clear irq */
	enable_isp_irq(hrt_isp_css_irq_sp, false);
	clear_isp_irq(hrt_isp_css_irq_sp);
	/*
	 * TODO: do we need to reset any other interrupts,
	 * i.e hrt_isp_css_irq_sw_1 or hrt_isp_css_irq_sw_2?
	 */
	/* stream off sensor */
	if (!isp->sw_contex.file_input) {
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 0);
		if (ret)
			dev_warn(isp->dev,
				 "can't stop streaming on sensor!\n");
	}

	/* reset ISP and restore its state */
	atomisp_reset(isp);

	isp->s3a_bufs_in_css = 0;
	isp->frame_bufs_in_css = 0;
	isp->dis_bufs_in_css = 0;
	isp->vf_frame_bufs_in_css = 0;

	isp->s3a_bufs_in_css = 0;
	isp->frame_bufs_in_css = 0;
	isp->dis_bufs_in_css = 0;
	isp->vf_frame_bufs_in_css = 0;

	sh_css_start(css_pipe_id);
	if (!isp->sw_contex.file_input) {
#ifndef CONFIG_X86_MRFLD
		sh_css_enable_interrupt(SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF, true);
#endif /* CONFIG_X86_MRFLD */
		atomisp_set_term_en_count(isp);
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 1);
		if (ret)
			dev_warn(isp->dev,
				 "can't start streaming on sensor!\n");
	}
}

/* 0x100000 is the start of dmem inside SP */
#define SP_DMEM_BASE	0x100000

void dump_sp_dmem(unsigned int addr, unsigned int size)
{
	unsigned int data = 0;
	unsigned int size32 = (size + (sizeof(u32) - 1)) / sizeof(u32);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "atomisp_io_base:0x%x\n",(unsigned int)atomisp_io_base);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "%s, addr:0x%x, size: %d, size32: %d\n",
		 __func__, addr, size, size32);
	if (size32 * 4 + addr > 0x4000) {
		v4l2_err(&atomisp_dev,
			 "illegal size (%d) or addr (0x%x)\n",
			 size32, addr);
		return;
	}
	addr += SP_DMEM_BASE;
	do {
		data = _hrt_master_port_uload_32(addr);

		/* printk/dtrace */
#if 1
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "%s, \t [0x%x]:0x%x\n",
			 __func__, addr, data);
#else
		atomisp_dtrace(2,"%s, \t [0x%x]:0x%x\n",
				__func__, addr, data);
#endif
		addr += sizeof(unsigned int);
		size32 -= 1;
	} while(size32 > 0);
}

static void atomisp_timeout_handler(struct atomisp_device *isp)
{
	char debug_context[64];

	v4l2_err(&atomisp_dev, "ISP timeout\n");
	isp->isp_timeout = true;

	sh_css_dump_sp_sw_debug_info();
	snprintf(debug_context, 64, "ISP timeout encountered (%d of 5)",
		 atomic_read(&isp->wdt_count));
	sh_css_dump_debug_info(debug_context);
	v4l2_err(&atomisp_dev,
			"%s, frames in css: %d\n",
			__func__, isp->frame_bufs_in_css);
	v4l2_err(&atomisp_dev,
			"%s, vf frames in css: %d\n",
			__func__, isp->vf_frame_bufs_in_css);
	v4l2_err(&atomisp_dev,
			"%s, s3a buffers in css: %d\n",
			__func__, isp->s3a_bufs_in_css);
	v4l2_err(&atomisp_dev,
			"%s, dis buffers in css: %d\n",
			__func__, isp->dis_bufs_in_css);
	/*sh_css_dump_sp_state();*/
	/*sh_css_dump_isp_state();*/
	atomisp_pipe_reset(isp);

	/* dequeueing buffers is not needed. CSS will recycle buffers that it
	 * has. */
	atomisp_flush_bufs_and_wakeup(isp);
	v4l2_err(&atomisp_dev, "ISP timeout recovery handling done\n");
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
		if (handle->data == frame->data)
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

/* Returns queued buffers back to video-core */
void atomisp_flush_bufs_and_wakeup(struct atomisp_device *isp)
{
	struct atomisp_video_pipe *pipe;
	unsigned long irqflags;
	int i;

	pipe = &isp->isp_subdev.video_out_capture;
	if(pipe->opened) {
		for (i = 0; pipe->capq.bufs[i]; i++) {
			spin_lock_irqsave(&pipe->irq_lock, irqflags);
			if (pipe->capq.bufs[i]->state == VIDEOBUF_ACTIVE ||
					pipe->capq.bufs[i]->state == VIDEOBUF_QUEUED) {
				get_buf_timestamp(&pipe->capq.bufs[i]->ts);
				pipe->capq.bufs[i]->field_count =
					atomic_read(&isp->sequence) << 1;
				pipe->capq.bufs[i]->state = VIDEOBUF_ERROR;
				wake_up(&pipe->capq.bufs[i]->done);
				v4l2_dbg(2, dbg_level, &atomisp_dev,
					 "release buffer from capture queue\n");
			}
			spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
		}
	}

	pipe = &isp->isp_subdev.video_out_vf;
	if(pipe->opened) {
		for (i = 0; pipe->capq.bufs[i]; i++) {
			spin_lock_irqsave(&pipe->irq_lock, irqflags);
			if (pipe->capq.bufs[i]->state == VIDEOBUF_ACTIVE ||
					pipe->capq.bufs[i]->state == VIDEOBUF_QUEUED) {
				get_buf_timestamp(&pipe->capq.bufs[i]->ts);
				/* videobuf field count == sequence * 2 */
				pipe->capq.bufs[i]->field_count =
					atomic_read(&isp->sequence) << 1;
				pipe->capq.bufs[i]->state = VIDEOBUF_ERROR;
				wake_up(&pipe->capq.bufs[i]->done);
				v4l2_dbg(2, dbg_level, &atomisp_dev,
					 "release buffer from vf queue\n");
			}
			spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
		}
	}

	pipe = &isp->isp_subdev.video_out_preview;
	if(pipe->opened) {
		for (i = 0; pipe->capq.bufs[i]; i++) {
			spin_lock_irqsave(&pipe->irq_lock, irqflags);
			if (pipe->capq.bufs[i]->state == VIDEOBUF_ACTIVE ||
					pipe->capq.bufs[i]->state == VIDEOBUF_QUEUED) {
				get_buf_timestamp(&pipe->capq.bufs[i]->ts);
				pipe->capq.bufs[i]->field_count =
					atomic_read(&isp->sequence) << 1;
				pipe->capq.bufs[i]->state = VIDEOBUF_ERROR;
				wake_up(&pipe->capq.bufs[i]->done);
				v4l2_dbg(2, dbg_level, &atomisp_dev,
					 "release buffer from preview queue\n");
			}
			spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
		}
	}
}

static void atomisp_buf_done(struct atomisp_device *isp,
		int error, enum sh_css_buffer_type buf_type)
{
	struct videobuf_buffer *vb = NULL;
	struct atomisp_video_pipe *pipe;
	void *buffer;
	enum sh_css_pipe_id css_pipe_id;
	bool requeue = false;
	int err;
	unsigned long irqflags;
	struct sh_css_frame* frame = NULL;

	/* choose right pipe for atomisp_qbuffers_to_css() main output */
	if (isp->sw_contex.run_mode != CI_MODE_PREVIEW)
		pipe = &isp->isp_subdev.video_out_capture;
	else
		pipe = &isp->isp_subdev.video_out_preview;

	err = atomisp_get_css_pipe_id(isp, &css_pipe_id);
	if (err < 0)
		return;

	if (buf_type != SH_CSS_BUFFER_TYPE_3A_STATISTICS &&
	    buf_type != SH_CSS_BUFFER_TYPE_DIS_STATISTICS &&
	    buf_type != SH_CSS_BUFFER_TYPE_OUTPUT_FRAME &&
	    buf_type != SH_CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME &&
	    buf_type != SH_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME) {
		v4l2_err(&atomisp_dev,
			"%s, unsupported buffer type: %d\n",
			__func__, buf_type);
		return;
	}

	err = sh_css_dequeue_buffer(css_pipe_id,
			buf_type,
			(void **)&buffer);
	if (err){
		v4l2_err(&atomisp_dev,
			"sh_css_dequeue_buffer failed: 0x%x\n",
			err);
		return;
	}
	switch (buf_type) {
		case SH_CSS_BUFFER_TYPE_3A_STATISTICS:
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
				err = sh_css_get_3a_statistics(
					isp->params.s3a_output_buf,
					isp->params.curr_grid_info.s3a_grid.use_dmem,
					buffer);
				if (err == sh_css_success)
					isp->params.s3a_buf_data_valid = true;
				else
					v4l2_err(&atomisp_dev,
						"get 3a statistics failed, not enough memory\n");
			}
			isp->s3a_bufs_in_css--;
			break;
		case SH_CSS_BUFFER_TYPE_DIS_STATISTICS:
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
				sh_css_get_dis_projections(
					isp->params.dis_hor_proj_buf,
					isp->params.dis_ver_proj_buf, buffer);

				isp->params.dis_proj_data_valid = true;
				/* wake up the sleep thread in atomisp_get_dis_stat */
				complete(&isp->dis_state_complete);
			}
			isp->dis_bufs_in_css--;
			break;
		case SH_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME:
			if (isp->sw_contex.invalid_vf_frame) {
				error = true;
				isp->sw_contex.invalid_vf_frame = 0;
				v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "%s css has marked this vf frame as invalid\n",
				 __func__);
			}
			if (!atomisp_is_viewfinder_support(isp)) {
				v4l2_err(&atomisp_dev,
					 "%s: viewfinder is not enabled!\n",
					 __func__);
				return;
			}
			/* relay buffer to correct pipe */
			if (isp->sw_contex.run_mode == CI_MODE_STILL_CAPTURE)
				pipe = &isp->isp_subdev.video_out_vf;
			else
				pipe = &isp->isp_subdev.video_out_preview;
			isp->vf_frame_bufs_in_css--;
			frame = buffer;
			if (isp->params.flash_state == ATOMISP_FLASH_ONGOING) {
				if (frame->flash_state
				    == SH_CSS_FRAME_PARTIAL_FLASH)
					v4l2_dbg(3, dbg_level, &atomisp_dev, "%s thumb partially flashed\n",__func__);
				else if (frame->flash_state
					 == SH_CSS_FRAME_FULLY_FLASH)
					v4l2_dbg(3, dbg_level, &atomisp_dev, "%s thumb completely flashed\n",__func__);
				else
					v4l2_dbg(3, dbg_level, &atomisp_dev, "%s thumb no flash in this frame\n",__func__);
			}
			vb = atomisp_css_frame_to_vbuf(pipe, buffer);
			if (!vb)
				v4l2_err(&atomisp_dev,
						"dequeued frame unknown!");
			break;
		case SH_CSS_BUFFER_TYPE_OUTPUT_FRAME:
			if (isp->sw_contex.invalid_frame) {
				error = true;
				isp->sw_contex.invalid_frame = 0;
				v4l2_dbg(3, dbg_level, &atomisp_dev,
				   "%s css has marked this frame as invalid\n",
				   __func__);
			}
			/* relay buffer to correct pipe */
			if (isp->sw_contex.run_mode != CI_MODE_PREVIEW)
				pipe = &isp->isp_subdev.video_out_capture;
			else
				pipe = &isp->isp_subdev.video_out_preview;
			isp->frame_bufs_in_css--;
			vb = atomisp_css_frame_to_vbuf(pipe, buffer);
			frame = buffer;

			if (isp->params.flash_state == ATOMISP_FLASH_ONGOING) {
				if (frame->flash_state
				    == SH_CSS_FRAME_PARTIAL_FLASH) {
					isp->frame_status[vb->i] =
						ATOMISP_FRAME_STATUS_FLASH_PARTIAL;
					v4l2_dbg(3, dbg_level, &atomisp_dev,
						 "%s partially flashed\n",
						 __func__);
				} else if (frame->flash_state
					   == SH_CSS_FRAME_FULLY_FLASH) {
					isp->frame_status[vb->i] =
						ATOMISP_FRAME_STATUS_FLASH_EXPOSED;
					isp->params.num_flash_frames--;
					v4l2_dbg(3, dbg_level, &atomisp_dev,
						 "%s completely flashed\n",
						 __func__);
				} else {
					isp->frame_status[vb->i] =
						ATOMISP_FRAME_STATUS_OK;
					v4l2_dbg(3, dbg_level, &atomisp_dev,
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

			if (!vb)
				v4l2_err(&atomisp_dev,
						"dequeued frame unknown!");
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
		err = sh_css_queue_buffer(css_pipe_id, buf_type, buffer);
		if (err)
			v4l2_err(&atomisp_dev,"%s, q to css fails: %d\n",
					__func__, err);
		return;
	}
	if (!error)
		atomisp_qbuffers_to_css(isp, pipe);
}

void atomisp_wdt_work(struct work_struct *work)
{
	struct atomisp_device *isp = container_of(work, struct atomisp_device,
						  wdt_work);

	mutex_lock(&isp->mutex);
	switch (atomic_inc_return(&isp->wdt_count)) {
	case ATOMISP_ISP_MAX_TIMEOUT_COUNT:
		isp->sw_contex.error = true;

		isp->s3a_bufs_in_css = 0;
		isp->frame_bufs_in_css = 0;
		isp->dis_bufs_in_css = 0;
		isp->vf_frame_bufs_in_css = 0;

		atomic_set(&isp->wdt_count, 0);

		atomisp_flush_bufs_and_wakeup(isp);
		break;
	default:
		atomisp_timeout_handler(isp);
	}

	mod_timer(&isp->wdt, jiffies + ATOMISP_ISP_TIMEOUT_DURATION);
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
			v4l2_err(&atomisp_dev, "flash timeout configure failed\n");
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
	uint32_t cssEvent = 0;
	uint32_t cssPipeId = 0;
	int s3aCount = 0;
	int frameCount = 0;
	int vfCount = 0;
	int disCount = 0;
	const int MAX_EVENTS = 10;
	enum sh_css_buffer_type events[MAX_EVENTS];
	int next_event = 0;
	int current_event = 0;
	bool frame_done_found = false;

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s\n", __func__);
	mutex_lock(&isp->mutex);

	while (isp->sw_contex.isp_streaming &&
	       sh_css_dequeue_event(&cssPipeId, &cssEvent) == sh_css_success) {
		events[next_event] = cssEvent;
		next_event = (next_event + 1) % MAX_EVENTS;
		switch (cssEvent) {
		case SH_CSS_EVENT_OUTPUT_FRAME_DONE:
			frameCount++;
			break;
		case SH_CSS_EVENT_3A_STATISTICS_DONE:
			s3aCount++;
			break;
		case SH_CSS_EVENT_VF_OUTPUT_FRAME_DONE:
			vfCount++;
			break;
		case SH_CSS_EVENT_DIS_STATISTICS_DONE:
			disCount++;
			break;
		case SH_CSS_EVENT_PIPELINE_DONE:
			break;
		default:
			v4l2_err(&atomisp_dev,
				 "unhandled css event: 0x%x\n",
				 cssEvent & 0xFFFF);
			break;
		}
	}

	while (isp->sw_contex.isp_streaming &&
	       current_event != next_event) {
		switch (events[current_event]) {
		case SH_CSS_EVENT_OUTPUT_FRAME_DONE:
			frame_done_found = true;
			atomisp_buf_done(
				isp, 0,
				SH_CSS_BUFFER_TYPE_OUTPUT_FRAME);
			frameCount--;
			break;
		case SH_CSS_EVENT_3A_STATISTICS_DONE:
			atomisp_buf_done(
				isp, 0,
				SH_CSS_BUFFER_TYPE_3A_STATISTICS);
			s3aCount--;
			break;
		case SH_CSS_EVENT_VF_OUTPUT_FRAME_DONE:
			atomisp_buf_done(
				isp, 0,
				SH_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME);
			vfCount--;
			break;
		case SH_CSS_EVENT_DIS_STATISTICS_DONE:
			atomisp_buf_done(
				isp, 0,
				SH_CSS_BUFFER_TYPE_DIS_STATISTICS);
			disCount--;
			break;
		case SH_CSS_EVENT_PIPELINE_DONE:
			break;
		default:
			v4l2_err(&atomisp_dev,
				 "unhandled css stored event: 0x%x\n",
				 events[current_event]);
			break;
		}
		events[current_event] = SH_CSS_NR_OF_EVENTS;

		current_event = (current_event + 1) % MAX_EVENTS;
	}

	if (isp->sw_contex.isp_streaming &&
	    frame_done_found &&
	    isp->params.css_update_params_needed) {
		sh_css_update_isp_params();
		isp->params.css_update_params_needed = false;
		frame_done_found = false;
	}
	isp->isp_timeout = false;
	atomisp_setup_flash(isp);
	mutex_unlock(&isp->mutex);

	v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s\n", __func__);

	return IRQ_HANDLED;
}

/*
 * utils for buffer allocation/free
 */

int atomisp_get_frame_pgnr(const struct sh_css_frame *frame, u32 *p_pgnr)
{
	if (!frame) {
		v4l2_err(&atomisp_dev,
			    "%s: NULL frame pointer ERROR.\n",
			    __func__);
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
/*
 * 2 types of format: SH format, v4l2 format
 * atomisp_format_bridge is a wrapper format for the other 2
 */
static const struct atomisp_format_bridge *get_atomisp_format_bridge(
					unsigned int pixelformat)
{
	unsigned int i;

	for (i = 0; i < atomisp_output_fmts_num; i++) {
		if (atomisp_output_fmts[i].pixelformat == pixelformat)
			return &atomisp_output_fmts[i];
	}

	return NULL;
}

static const struct atomisp_format_bridge *get_atomisp_format_bridge_from_mbus(
				enum v4l2_mbus_pixelcode mbus_code)
{
	unsigned int i;

	for (i = 0; i < atomisp_output_fmts_num; i++) {
		if (atomisp_output_fmts[i].mbus_code == mbus_code)
			return &atomisp_output_fmts[i];
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
	unsigned int i;

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

	if (isp->sw_contex.run_mode == CI_MODE_STILL_CAPTURE &&
	    isp->capture_format &&
	    isp->capture_format->out_sh_fmt == SH_CSS_FRAME_FORMAT_RAW &&
	    isp->sw_contex.bypass)
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
	if (isp->params.low_light)
		sh_css_capture_set_mode(SH_CSS_CAPTURE_MODE_LOW_LIGHT);
	else if (isp->params.gdc_cac_en)
		sh_css_capture_set_mode(SH_CSS_CAPTURE_MODE_ADVANCED);
	else
		sh_css_capture_set_mode(SH_CSS_CAPTURE_MODE_PRIMARY);
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
	if (!xnr_enable)
		return 0;

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
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.tnr_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
 * Function to get histogram data for image frame
 */
int atomisp_histogram(struct atomisp_device *isp, int flag, void *config)
{
#if defined(CONFIG_CSS_ONE)
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
#endif
	return 0;
}

/*
 * Function to configure black level compensation
 */
int atomisp_black_level(struct atomisp_device *isp, int flag,
			struct atomisp_ob_config *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.ob_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.ee_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.gamma_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.ctc_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
	if (sizeof(*config) != sizeof(isp->params.gc_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
#if 0
	/* TODO: check overlay functionality with css 1.5 */
	if (isp->params.vf_overlay) {
		if (isp->params.vf_overlay->frame)
			sh_css_frame_free(isp->params.vf_overlay->frame);
		kfree(isp->params.vf_overlay);
		isp->params.vf_overlay = NULL;
	}
#endif
	if (isp->raw_output_frame) {
		sh_css_frame_free(isp->raw_output_frame);
		isp->raw_output_frame = NULL;
	}
}

void atomisp_free_3a_dis_buffers(struct atomisp_device *isp)
{
	struct atomisp_s3a_buf *s3a_buf;
	struct atomisp_dis_buf *dis_buf;

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

	while (!list_empty(&isp->s3a_stats)) {
		s3a_buf = list_first_entry(&isp->s3a_stats,
				struct atomisp_s3a_buf, list);
		sh_css_free_stat_buffers(&s3a_buf->s3a_data,
					 NULL);
		list_del(&s3a_buf->list);
		kfree(s3a_buf);
	}

	while (!list_empty(&isp->dis_stats)) {
		dis_buf = list_entry(isp->dis_stats.next,
				struct atomisp_dis_buf, list);
		sh_css_free_stat_buffers(NULL,
					 &dis_buf->dis_data);
		list_del(&dis_buf->list);
		kfree(dis_buf);
	}
}

static void atomisp_update_grid_info(struct atomisp_device *isp)
{
	int err;
	struct sh_css_grid_info old_info = isp->params.curr_grid_info;
	switch (isp->sw_contex.run_mode) {
	case CI_MODE_PREVIEW:
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, preview\n",
			 __func__);
		err = sh_css_preview_get_grid_info(&isp->params.curr_grid_info);
		if(err)
			v4l2_err(&atomisp_dev,
				 "sh_css_capture_get_grid_info failed: %d\n",
				 err);
		/*err = sh_css_preview_get_s3a_dis_info(&isp->stat_info);*/
		if (err) {
			v4l2_err(&atomisp_dev,
				 "sh_css_capture_get_s3a_dis_info failed: %d\n",
				 err);
		}
		break;
	case CI_MODE_VIDEO:
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, video\n", __func__);
		err = sh_css_video_get_grid_info(&isp->params.curr_grid_info);
		if (err)
			v4l2_err(&atomisp_dev,
				 "sh_css_capture_get_grid_info failed: %d\n",
				 err);
		/*err = sh_css_video_get_s3a_dis_info(&isp->stat_info);*/
		if (err) {
			v4l2_err(&atomisp_dev,
				 "sh_css_capture_get_s3a_dis_info failed: %d\n",
				 err);
		}
		break;
	default:
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, default(capture)\n",
			 __func__);
		err = sh_css_capture_get_grid_info(&isp->params.curr_grid_info);
		if (err)
			v4l2_err(&atomisp_dev,
				 "sh_css_capture_get_grid_info failed: %d\n",
				 err);
		/*err = sh_css_capture_get_s3a_dis_info(&isp->stat_info);*/
		if (err) {
			v4l2_err(&atomisp_dev,
				 "sh_css_capture_get_s3a_dis_info failed: %d\n",
				 err);
		}
		break;
	}
	/* If the grid info has changed, we need to reallocate
	   the buffers for 3A and DIS statistics. */
	if (memcmp(&old_info, &isp->params.curr_grid_info, sizeof(old_info)) ||
	    !isp->params.s3a_output_buf || !isp->params.dis_hor_coef_buf) {
		/* We must free all buffers because they no longer match
		   the grid size. */
		atomisp_free_3a_dis_buffers(isp);

		err = atomisp_alloc_css_stat_bufs(isp, ATOMISP_CSS_Q_DEPTH);
		if (err) {
			v4l2_err(&atomisp_dev,
					"stat_buf allocate error\n");
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
		isp->params.s3a_output_buf = vmalloc(
				isp->params.s3a_output_bytes);

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

		isp->params.dis_hor_proj_buf =
			kzalloc(isp->params.dis_hor_proj_bytes, GFP_KERNEL);
		if (isp->params.dis_hor_proj_buf == NULL &&
		    isp->params.dis_hor_proj_bytes != 0)
			goto err_dis;

		isp->params.dis_ver_proj_buf =
			kzalloc(isp->params.dis_ver_proj_bytes, GFP_KERNEL);
		if (isp->params.dis_ver_proj_buf == NULL &&
		    isp->params.dis_ver_proj_bytes != 0)
			goto err_dis;
	}
	return;

	/* Failure for 3A buffers does not influence DIS buffers */
err_3a:
	if (isp->params.s3a_output_bytes != 0) {
		/* For SOC sensor happens s3a_output_bytes == 0,
		*  using if condition to exclude false error log
		*/
		v4l2_err(&atomisp_dev,
			    "Failed allocate memory for 3A statistics\n");
	}
	atomisp_free_3a_dis_buffers(isp);
	return;

err_dis:
	v4l2_err(&atomisp_dev,
		    "Failed allocate memory for DIS statistics\n");
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
				v4l2_err(&atomisp_dev,
					"Failed to copy to User for x\n");
				return -EFAULT;
			}
			ret = copy_to_user(config->coordinates_y[i],
				tab->coordinates_y[i], tab->height *
				tab->width * sizeof(*tab->coordinates_y[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
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
			v4l2_err(&atomisp_dev, "out of memory\n");
			return -EINVAL;
		}

		for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			ret = copy_from_user(tab->coordinates_x[i],
				config->coordinates_x[i],
				config->height * config->width *
				sizeof(*config->coordinates_x[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
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
				v4l2_err(&atomisp_dev,
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

	if (config == NULL)
		return -EINVAL;

	if (sizeof(config->table) != sizeof(*macc_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
	int error;
	long left;

	if (stats->vertical_projections   == NULL ||
	    stats->horizontal_projections == NULL ||
	    isp->params.dis_hor_proj_buf  == NULL ||
	    isp->params.dis_ver_proj_buf  == NULL)
		return -EINVAL;

	/* isp needs to be streaming to get DIS statistics */
	if (!isp->sw_contex.isp_streaming)
		return -EINVAL;
	if (!isp->params.video_dis_en)
		return -EINVAL;

	INIT_COMPLETION(isp->dis_state_complete);
	/*this might be blocking other v4l2 calls*/
	if(!isp->params.dis_proj_data_valid) {
		/* ioctl is holding lock when this is called */
		mutex_unlock(&isp->mutex);
		left = wait_for_completion_timeout(&isp->dis_state_complete,
						   1 * HZ);
		mutex_lock(&isp->mutex);

		/* Timeout to get the statistics */
		if (left == 0) {
			v4l2_err(&atomisp_dev,
				 "Failed to wait frame DIS state\n");
			return -EINVAL;
		}
		if (!isp->params.dis_proj_data_valid) {
			v4l2_err(&atomisp_dev,
			"%s: dis_proj_data_valid is not true, try again\n",
					__func__);
			return -EAGAIN;
		}
	}

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

static int atomisp_compare_grid(struct atomisp_device *isp,
				struct atomisp_grid_info *atomgrid)
{
	struct atomisp_grid_info tmp;
	memset(&tmp, 0, sizeof(tmp));
	atomisp_curr_user_grid_info(isp, &tmp);
	return memcmp(atomgrid, &tmp, sizeof(tmp));
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

	if (config == NULL)
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
		v4l2_err(&atomisp_dev, "3a statistics is not valid.\n");
		return -EAGAIN;
	}

	ret = copy_to_user(config->data,
			   isp->params.s3a_output_buf,
			   isp->params.s3a_output_bytes);
	if (ret) {
		v4l2_err(&atomisp_dev,
			    "copy to user failed: copied %lu bytes\n", ret);
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
	/* Read parameter for 3A bianry info */
	if (flag == 0) {
		if (&config->info == NULL) {
			v4l2_err(&atomisp_dev,
				    "ERROR: NULL pointer in grid_info\n");
			return -EINVAL;
		}
		atomisp_curr_user_grid_info(isp, &config->info);
		return 0;
	}

	if (sizeof(config->wb_config) != sizeof(isp->params.wb_config))
		goto INVALID_PARM;
	if (sizeof(config->cc_config) != sizeof(isp->params.cc_config))
		goto INVALID_PARM;
	if (sizeof(config->ob_config) != sizeof(isp->params.ob_config))
		goto INVALID_PARM;
	if (sizeof(config->de_config) != sizeof(isp->params.de_config))
		goto INVALID_PARM;
	if (sizeof(config->ce_config) != sizeof(isp->params.ce_config))
		goto INVALID_PARM;
	if (sizeof(config->dp_config) != sizeof(isp->params.dp_config))
		goto INVALID_PARM;
	if (sizeof(config->nr_config) != sizeof(isp->params.nr_config))
		goto INVALID_PARM;
	if (sizeof(config->ee_config) != sizeof(isp->params.ee_config))
		goto INVALID_PARM;
	if (sizeof(config->tnr_config) != sizeof(isp->params.tnr_config))
		goto INVALID_PARM;

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
	const struct sh_css_cc_config *cc_config;
	const struct sh_css_macc_table *macc_table;
	const struct sh_css_ctc_table *ctc_table;

	if (flag == 0) {
		*effect = isp->params.color_effect;
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
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.dp_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
 * Function to configure vf overlay image
 */
int atomisp_vf_overlay(struct atomisp_device *isp, int flag,
		       struct atomisp_overlay *arg)
{
	int ret = 0;

	if (arg == NULL)
		return -EINVAL;
#if 0	/* TODO: Check overlay functionality with css1.5 */
	/* NULL means disable the feature */
	if (!arg->frame) {
		sh_css_overlay_set_for_viewfinder(NULL);
		return 0;
	}

	if (isp->params.vf_overlay) {
		if (isp->params.vf_overlay->frame)
			sh_css_frame_free(isp->params.vf_overlay->frame);
		kfree(isp->params.vf_overlay);
	}

	isp->params.vf_overlay = kzalloc(sizeof(struct sh_css_overlay),
							GFP_KERNEL);
	if (!isp->params.vf_overlay) {
		ret =  -ENOMEM;
		goto err;
	}

	ret = atomisp_v4l2_framebuffer_to_sh_css_frame(arg->frame,
						&isp->params.vf_overlay->frame);
	if (ret)
		goto err;

	isp->params.vf_overlay->bg_y               = arg->bg_y;
	isp->params.vf_overlay->bg_u               = arg->bg_u;
	isp->params.vf_overlay->bg_v               = arg->bg_v;
	isp->params.vf_overlay->blend_input_perc_y = arg->blend_input_perc_y;
	isp->params.vf_overlay->blend_input_perc_u = arg->blend_input_perc_u;
	isp->params.vf_overlay->blend_input_perc_v = arg->blend_input_perc_v;
	isp->params.vf_overlay->blend_overlay_perc_y =
						arg->blend_overlay_perc_y;
	isp->params.vf_overlay->blend_overlay_perc_u =
						arg->blend_overlay_perc_u;
	isp->params.vf_overlay->blend_overlay_perc_v =
						arg->blend_overlay_perc_v;
	isp->params.vf_overlay->overlay_start_x    = arg->overlay_start_x;
	isp->params.vf_overlay->overlay_start_y    = arg->overlay_start_y;

	sh_css_overlay_set_for_viewfinder(isp->params.vf_overlay);

err:
	if (ret && isp->params.vf_overlay)
		kfree(isp->params.vf_overlay);
#endif
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
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.de_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.wb_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

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
	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s %d\n", __func__, flag);
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp->params.s3a_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get white balance from current setup */
		memcpy(config, &isp->params.s3a_config, sizeof(*config));
	} else {
		/* Set white balance to isp parameters */
		memcpy(&isp->params.s3a_config, config, sizeof(*config));
		/*
		 * using default values for awb threshold valued
		 * until 3a library starts setting values by itself
		 */
		isp->params.s3a_config.awb_lg_high_raw = 1022;
		isp->params.s3a_config.awb_lg_low = 1;
		isp->params.s3a_config.awb_lg_high = 8191;
		sh_css_set_3a_config(&isp->params.s3a_config);
		isp->params.css_update_params_needed = true;
		/* isp->params.s3a_buf_data_valid = false; */
	}

	v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s %d\n", __func__, flag);
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
		IS_MRFLD ? MRFLD_MAX_ZOOM_FACTOR : MFLD_MAX_ZOOM_FACTOR;

	if (flag == 0) {
		sh_css_get_zoom_factor(&zoom, &zoom);
		*value = max_zoom - zoom;
	} else {
		if (*value < 0)
			return -EINVAL;

		zoom = max_zoom - min_t(u32, max_zoom, (*value));

		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, zoom: %d\n",
			 __func__, zoom);
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
	if (pipe->format && pipe->format->out.width != 0) {
		memcpy(&f->fmt.pix, &pipe->format->out,
			sizeof(struct v4l2_pix_format));
	} else {
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
	const struct atomisp_format_bridge *fmt;
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
	snr_mbus_fmt.height = out_height;
	snr_mbus_fmt.width = out_width;

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			video, try_mbus_fmt, &snr_mbus_fmt);
	if (ret)
		return ret;

	in_width = snr_mbus_fmt.width;
	in_height = snr_mbus_fmt.height;
	fmt = get_atomisp_format_bridge_from_mbus(snr_mbus_fmt.code);
	if (fmt == NULL) {
		f->fmt.pix.pixelformat = pixelformat;
		v4l2_err(&atomisp_dev, "unknown sensor format.\n");
	} else {
		f->fmt.pix.pixelformat = fmt->pixelformat;
	}

	if (in_width < out_width && in_height < out_height) {
		out_width = in_width;
		out_height = in_height;
		/* Set the flag when resolution requested is
		 * beyond the max value supported by sensor
		 */
		if (res_overflow != NULL)
			*res_overflow = true;
	}

	/* app vs isp */
	out_width = min_t(u32, out_width, ATOM_ISP_MAX_WIDTH);
	out_height = min_t(u32, out_height, ATOM_ISP_MAX_HEIGHT);

	out_width = max_t(u32, out_width, ATOM_ISP_MIN_WIDTH);
	out_height = max_t(u32, out_height, ATOM_ISP_MIN_HEIGHT);

	out_width = out_width - out_width % ATOM_ISP_STEP_WIDTH;
	out_height = out_height - out_height % ATOM_ISP_STEP_HEIGHT;

	f->fmt.pix.width = out_width;
	f->fmt.pix.height = out_height;

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
	f->fmt.pix.width = clamp_t(u32,
				   rounddown(width, (u32)ATOM_ISP_STEP_WIDTH),
				   ATOM_ISP_MIN_WIDTH, ATOM_ISP_MAX_WIDTH);
	f->fmt.pix.height = clamp_t(u32, rounddown(height,
						   (u32)ATOM_ISP_STEP_HEIGHT),
				    ATOM_ISP_MIN_HEIGHT, ATOM_ISP_MAX_HEIGHT);
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
static mipi_port_ID_t __get_mipi_port(enum atomisp_camera_port port)
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
		v4l2_err(&atomisp_dev, "unsupported port: %d\n", port);
		return MIPI_PORT0_ID;
	}
}

static inline void
atomisp_set_sensor_mipi_to_isp(struct camera_mipi_info *mipi_info)
{
	if (mipi_info) {
		if (atomisp_input_format_is_raw(mipi_info->input_format))
			sh_css_input_set_bayer_order(
				mipi_info->raw_bayer_order);
		sh_css_input_set_format((enum sh_css_input_format)
					mipi_info->input_format);
		sh_css_input_configure_port(__get_mipi_port(mipi_info->port),
					    mipi_info->num_lanes,
					    0xffff4);
	} else {
		/*Use default MIPI configuration*/
		sh_css_input_set_bayer_order(sh_css_bayer_order_grbg);
		sh_css_input_set_format(SH_CSS_INPUT_FORMAT_RAW_10);
		sh_css_input_configure_port(
				__get_mipi_port(ATOMISP_CAMERA_PORT_PRIMARY),
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
	const struct atomisp_format_bridge *format;
	int effective_input_width = pipe->format->in.width;
	int effective_input_height = pipe->format->in.height;
	int ret;

	format = get_atomisp_format_bridge(pixelformat);
	if (format == NULL)
		return -EINVAL;

	isp->capture_format->out_sh_fmt = format->sh_fmt;
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

	v4l2_dbg(2, dbg_level, &atomisp_dev,
			"sh css input width: %d, height: %d\n",
			isp->input_format->out.width,
			isp->input_format->out.height);

	if (sh_css_input_set_effective_resolution(effective_input_width,
						  effective_input_height))
		return -EINVAL;

	v4l2_dbg(2, dbg_level, &atomisp_dev,
			"sh css input effective width: %d, height: %d\n",
			effective_input_width, effective_input_height);

	if (!isp->vf_format)
		isp->vf_format =
			kzalloc(sizeof(struct atomisp_video_pipe_format),
							GFP_KERNEL);
	if (!isp->vf_format) {
		v4l2_err(&atomisp_dev, "Failed to alloc preview_format memory\n");
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

		v4l2_dbg(3, dbg_level, &atomisp_dev,
					"sh css preview output width: %d, height: %d\n",
					width, height);

		if (sh_css_preview_get_output_frame_info(output_info))
			return -EINVAL;

		v4l2_dbg(3, dbg_level, &atomisp_dev,
				    "sh css preview output_info width: %d, height: %d\n",
					output_info->width, output_info->height);
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
		v4l2_dbg(3, dbg_level, &atomisp_dev,
					"sh css capture vf output width: %d, height: %d\n",
					isp->vf_format->out.width, isp->vf_format->out.height);

		if (sh_css_capture_configure_output(width, height,
						    format->sh_fmt))
			return -EINVAL;
		v4l2_dbg(3, dbg_level, &atomisp_dev,
					"sh css capture main output width: %d, height: %d\n",
					width, height);

		ret = sh_css_capture_get_output_frame_info(output_info);
		if (ret) {
			v4l2_err(&atomisp_dev,
				    "Resolution set mismatach error %d\n",
				    ret);
			return -EINVAL;
		}
		v4l2_dbg(3, dbg_level, &atomisp_dev,
					"sh css capture main output_info width: %d, height: %d\n",
					output_info->width, output_info->height);

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

	if (!isp->params.online_process && !isp->sw_contex.file_input
	    && sh_css_frame_allocate_from_info(&isp->raw_output_frame,
					       raw_output_info))
		return -ENOMEM;

	return 0;
}

static int atomisp_get_effective_resolution(struct atomisp_device *isp,
					    unsigned int pixelformat,
					    int out_width, int out_height,
					    int padding_w, int padding_h)
{
	const struct atomisp_format_bridge *format;
	struct v4l2_pix_format *in_fmt = &isp->capture_format->in;
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
		in_fmt->height = DIV_ROUND_UP(in_fmt->width * out_height,
					      out_width);
		if (in_fmt->height > no_padding_h) {
			in_fmt->height = no_padding_h;
			in_fmt->width = DIV_ROUND_UP(in_fmt->height * out_width,
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
		*dvs_env_w = *dvs_env_w - *dvs_env_w % ATOM_ISP_STEP_WIDTH;
		*dvs_env_h = *dvs_env_h - *dvs_env_h % ATOM_ISP_STEP_HEIGHT;
		sh_css_video_set_dis_envelope(*dvs_env_w, *dvs_env_h);
	} else {
		/* if DVS gets disabled, make sure it's indeed turned off */
		sh_css_video_set_dis_envelope(0, 0);
	}

	isp->params.dis_proj_data_valid = false;
	isp->params.css_update_params_needed = true;
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
	if (ret || hbin != vbin)
		hbin = 0;

	return hbin;
}

static int atomisp_set_fmt_to_snr(struct atomisp_device *isp,
			  struct v4l2_format *f, unsigned int pixelformat,
			  unsigned int padding_w, unsigned int padding_h,
			  unsigned int dvs_env_w, unsigned int dvs_env_h)
{
	const struct atomisp_format_bridge *format;
	struct v4l2_mbus_framefmt snr_mbus_fmt;
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

		v4l2_dbg(2, dbg_level, &atomisp_dev,
			 "sensor width: %d, height: %d\n",
			 snr_mbus_fmt.width, snr_mbus_fmt.height);
		isp->input_format->out.width = snr_mbus_fmt.width;
		isp->input_format->out.height = snr_mbus_fmt.height;
		isp->input_format->out.pixelformat =
		    snr_mbus_fmt.code;
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
static void atomisp_get_yuv_ds_status(struct atomisp_device *isp,
				      unsigned int width, unsigned int height)
{
	/* no YUV downscaling if sensor output is 10% larger than isp output */
	unsigned int w_tmp = isp->input_format->out.width -
		DIV_ROUND_UP(isp->input_format->out.width, 10);
	unsigned int h_tmp = isp->input_format->out.height -
		DIV_ROUND_UP(isp->input_format->out.height, 10);
	/*
	 * yuv downscaling is not enabled in video binary,
	 * ,raw format output, soc sensor. effective resolution should
	 * be the same as isp output.
	 * yuv-ds is enabled only when sensor output width is 10% larger
	 * than isp output width and sensor output height also is 10%
	 * larger than isp output height.
	 */
	if ((w_tmp < width || h_tmp < height)
		|| isp->sw_contex.run_mode == CI_MODE_VIDEO
		|| isp->sw_contex.bypass
		|| isp->sw_contex.file_input)
		isp->params.yuv_ds_en = false;
	else
		isp->params.yuv_ds_en = true;
}
int atomisp_set_fmt(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	const struct atomisp_format_bridge *format_bridge;
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

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    f->type != V4L2_BUF_TYPE_PRIVATE) {
		v4l2_err(&atomisp_dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	format_bridge = get_atomisp_format_bridge(pixelformat);
	if (format_bridge == NULL)
		return -EINVAL;

	sh_format = format_bridge->sh_fmt;

	if (pipe->pipe_type != ATOMISP_PIPE_CAPTURE &&
	    isp->sw_contex.run_mode != CI_MODE_PREVIEW) {
		/*
		 * Check whether VF resolution configured larger
		 * than Main Resolution. If so, Force VF resolution
		 * to be the same as Main resolution
		 */
		if (isp->isp_subdev.video_out_capture.format &&
		    isp->isp_subdev.video_out_capture.format->out.width &&
		    isp->isp_subdev.video_out_capture.format->out.height &&
		    (isp->isp_subdev.video_out_capture.format->out.width <
		     width ||
		     isp->isp_subdev.video_out_capture.format->out.height
		     < height)) {
			v4l2_warn(&atomisp_dev,
				  "Force capture resolution to same as "
				  "vf/preview\n");
			width = isp->isp_subdev.video_out_capture.format
				->out.width;
			height = isp->isp_subdev.video_out_capture.format
				->out.height;
		}

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
	/*
	 * Check whether main resolution configured smaller
	 * than snapshot resolution. If so, force main resolution
	 * to be the same as snapshot resolution
	 */
	if (pipe->pipe_type == ATOMISP_PIPE_CAPTURE &&
	    isp->isp_subdev.video_out_vf.format &&
	    isp->isp_subdev.video_out_vf.format->out.width &&
	    isp->isp_subdev.video_out_vf.format->out.height &&
	    (isp->isp_subdev.video_out_vf.format->out.width > width ||
	     isp->isp_subdev.video_out_vf.format->out.height > height)) {
		v4l2_warn(&atomisp_dev, "Main Resolution config "
			  "smaller then Vf Resolution. Force "
			  "to be equal with Vf Resolution.");
		width = isp->isp_subdev.video_out_vf.format->out.width;
		height = isp->isp_subdev.video_out_vf .format->out.height;
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
	} else {
		isp->sw_contex.bypass = false;
	}

	/* construct resolution supported by isp */
	if (res_overflow) {
		width -= padding_w;
		height -= padding_h;
		/* app vs isp */
		width = min_t(u32, width, ATOM_ISP_MAX_WIDTH);
		height = min_t(u32, height, ATOM_ISP_MAX_HEIGHT);

		width = max_t(u32, width, ATOM_ISP_MIN_WIDTH);
		height = max_t(u32, height, ATOM_ISP_MIN_HEIGHT);

		width = width - width % ATOM_ISP_STEP_WIDTH;
		height = height - height % ATOM_ISP_STEP_HEIGHT;

		f->fmt.pix.width = width;
		f->fmt.pix.height = height;
	}

	/* set dis envelop if video and dis are enabled */
	atomisp_set_dis_envelop(isp, width, height, &dvs_env_w, &dvs_env_h);

	if (!isp->input_format)
		isp->input_format =
			kzalloc(sizeof(struct atomisp_video_pipe_format),
				GFP_KERNEL);
	if (!isp->input_format) {
		v4l2_err(&atomisp_dev, "Failed to alloc input_format memory\n");
		return -ENOMEM;
	}

	/* set format info to sensor */
	ret = atomisp_set_fmt_to_snr(isp, f, pixelformat, padding_w, padding_h,
				     dvs_env_w, dvs_env_h);
	if (ret)
		return -EINVAL;

	/* Only main stream pipe will be here */
	isp->capture_format = pipe->format;
	if (!isp->capture_format) {
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
		DIV_ROUND_UP(format_bridge->depth * output_info.padded_width,
			     8);
	pipe->format->out.sizeimage =
	    PAGE_ALIGN(height * pipe->format->out.bytesperline);
	if (f->fmt.pix.field == V4L2_FIELD_ANY)
		f->fmt.pix.field = V4L2_FIELD_NONE;
	pipe->format->out.field = f->fmt.pix.field;
	pipe->format->out_sh_fmt = sh_format;

	memcpy(&f->fmt.pix, &pipe->format->out,
			sizeof(struct v4l2_pix_format));
	f->fmt.pix.priv = PAGE_ALIGN(pipe->format->out.width *
				     pipe->format->out.height * 2);

	pipe->capq.field = f->fmt.pix.field;

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
		sh_css_input_configure_port(
				__get_mipi_port(ATOMISP_CAMERA_PORT_PRIMARY),
						2, 0xffff4);
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

int atomisp_save_iunit_reg(struct atomisp_device *isp)
{
	struct pci_dev *dev = isp->pdev;

	v4l2_dbg(3, dbg_level, &atomisp_dev, "%s\n", __func__);

	pci_read_config_word(dev, PCI_COMMAND, &isp->hw_contex.pcicmdsts);
	pci_read_config_dword(dev, PCI_BASE_ADDRESS_0,
			      &isp->hw_contex.ispmmadr);
	pci_read_config_dword(dev, PCI_MSI_CAPID,
			      &isp->hw_contex.msicap);
	pci_read_config_dword(dev, PCI_MSI_ADDR,
			      &isp->hw_contex.msi_addr);
	pci_read_config_word(dev, PCI_MSI_DATA,
			     &isp->hw_contex.msi_data);
	pci_read_config_byte(dev, PCI_INTERRUPT_LINE,
			      &isp->hw_contex.intr);
	pci_read_config_dword(dev, PCI_INTERRUPT_CTRL,
			      &isp->hw_contex.interrupt_control);

	if (IS_MRFLD) {
		pci_read_config_dword(dev, MRFLD_PCI_PMCS,
				      &isp->hw_contex.pmcs);
		pci_read_config_dword(dev, MRFLD_PCI_CSI_ACCESS_CTRL_VIOL,
				      &isp->hw_contex.csi_access_viol);
		pci_read_config_dword(dev, MRFLD_PCI_CSI_RCOMP_CONTROL,
				      &isp->hw_contex.csi_rcomp_config);
		pci_read_config_dword(dev, MRFLD_PCI_CSI_AFE_TRIM_CONTROL,
				      &isp->hw_contex.csi_afe_dly);
		pci_read_config_dword(dev, MRFLD_PCI_CSI_CONTROL,
				      &isp->hw_contex.csi_control);
		pci_read_config_dword(dev, MRFLD_PCI_CSI_AFE_RCOMP_CONTROL,
				      &isp->hw_contex.csi_afe_rcomp_config);
		pci_read_config_dword(dev, MRFLD_PCI_CSI_AFE_HS_CONTROL,
				      &isp->hw_contex.csi_afe_hs_control);
		pci_read_config_dword(dev, MRFLD_PCI_CSI_DEADLINE_CONTROL,
				      &isp->hw_contex.csi_deadline_control);
	} else {
		pci_read_config_dword(dev, MFLD_PCI_PMCS,
				      &isp->hw_contex.pmcs);
		pci_read_config_dword(dev, MFLD_PCI_CG_DIS,
				      &isp->hw_contex.cg_dis);
		isp->hw_contex.csi_rcomp_config = intel_mid_msgbus_read32(
				MFLD_IUNITPHY_PORT, MFLD_CSI_RCOMP);
		isp->hw_contex.csi_afe_dly = intel_mid_msgbus_read32(
				MFLD_IUNITPHY_PORT, MFLD_CSI_AFE);
		isp->hw_contex.csi_control = intel_mid_msgbus_read32(
				MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL);
	}

	return 0;
}

int atomisp_restore_iunit_reg(struct atomisp_device *isp)
{
	struct pci_dev *dev = isp->pdev;
	u32 reg32;

	v4l2_dbg(3, dbg_level, &atomisp_dev, "%s\n", __func__);

	pci_write_config_word(dev, PCI_COMMAND, isp->hw_contex.pcicmdsts);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0,
			       isp->hw_contex.ispmmadr);
	pci_write_config_dword(dev, PCI_MSI_CAPID, isp->hw_contex.msicap);
	pci_write_config_dword(dev, PCI_MSI_ADDR, isp->hw_contex.msi_addr);
	pci_write_config_word(dev, PCI_MSI_DATA, isp->hw_contex.msi_data);
	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, isp->hw_contex.intr);
	pci_write_config_dword(dev, PCI_INTERRUPT_CTRL,
			       isp->hw_contex.interrupt_control);

	if (IS_MRFLD) {
		pci_write_config_dword(dev, MRFLD_PCI_PMCS,
						isp->hw_contex.pmcs);

		/*
		 * The default value is not 1 for all suported chips. Hence
		 * enable the read/write combining explicitly.
		 */
		pci_read_config_dword(dev, PCI_I_CONTROL, &reg32);
		reg32 |= MRFLD_PCI_I_CONTROL_ENABLE_READ_COMBINING
				| MRFLD_PCI_I_CONTROL_ENABLE_WRITE_COMBINING;
		pci_write_config_dword(dev, PCI_I_CONTROL, reg32);

		pci_write_config_dword(dev, MRFLD_PCI_CSI_ACCESS_CTRL_VIOL,
				      isp->hw_contex.csi_access_viol);
		pci_write_config_dword(dev, MRFLD_PCI_CSI_RCOMP_CONTROL,
				      isp->hw_contex.csi_rcomp_config);
		pci_write_config_dword(dev, MRFLD_PCI_CSI_AFE_TRIM_CONTROL,
				      isp->hw_contex.csi_afe_dly);
		pci_write_config_dword(dev, MRFLD_PCI_CSI_CONTROL,
				      isp->hw_contex.csi_control);
		pci_write_config_dword(dev, MRFLD_PCI_CSI_AFE_RCOMP_CONTROL,
				      isp->hw_contex.csi_afe_rcomp_config);
		pci_write_config_dword(dev, MRFLD_PCI_CSI_AFE_HS_CONTROL,
				      isp->hw_contex.csi_afe_hs_control);
		pci_write_config_dword(dev, MRFLD_PCI_CSI_DEADLINE_CONTROL,
				      isp->hw_contex.csi_deadline_control);

		/*
		 * for MRFLD, Software/firmware needs to write a 1 to bit0
		 * of the register at CSI_RECEIVER_SELECTION_REG to enable
		 * SH CSI backend write 0 will enable Arasan CSI backend,
		 * which has bugs(like sighting:4567697 and 4567699) and
		 * will be removed in B0
		 */
		device_store_uint32(MRFLD_CSI_RECEIVER_SELECTION_REG, 1);

	} else {
		pci_write_config_dword(dev, MFLD_PCI_PMCS, isp->hw_contex.pmcs);
		pci_write_config_dword(dev, MFLD_PCI_CG_DIS,
						isp->hw_contex.cg_dis);

		/*
		 * The default value is not 1 for all suported chips. Hence
		 * enable the read/write combining explicitly.
		 */
		pci_read_config_dword(dev, PCI_I_CONTROL, &reg32);
		reg32 |= MFLD_PCI_I_CONTROL_ENABLE_READ_COMBINING
				| MFLD_PCI_I_CONTROL_ENABLE_WRITE_COMBINING;
		pci_write_config_dword(dev, PCI_I_CONTROL, reg32);

		intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT, MFLD_CSI_RCOMP,
				    isp->hw_contex.csi_rcomp_config);
		intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT, MFLD_CSI_AFE,
				    isp->hw_contex.csi_afe_dly);
		intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL,
				    isp->hw_contex.csi_control);
	}

	return 0;
}

/*Turn off ISP dphy */
int atomisp_ospm_dphy_down(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;
	int timeout = 100;
	bool idle;
	v4l2_dbg(3, dbg_level, &atomisp_dev, "%s\n", __func__);

	/* MRFLD IUNIT PHY is located in an always-power-on island */
	if (IS_MRFLD)
		return 0;

	/* if ISP timeout, we can force powerdown */
	if (isp->isp_timeout) {
		isp->isp_timeout = false;
		goto done;
	}

	if (!isp->sw_contex.init)
		goto done;

	idle = sh_css_hrt_system_is_idle();
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "%s system_is_idle:%d\n", __func__, idle);
	while (!idle && timeout--) {
		udelay(20);
		idle = sh_css_hrt_system_is_idle();
	}

	if (timeout < 0) {
		v4l2_err(&atomisp_dev,
			 "Timeout to stop ISP HW\n");
		/* force power down here */
		/* return -EINVAL; */
	}

done:
	/* power down DPHY */
	pwr_cnt = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL);
	pwr_cnt |= 0x300;
	intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL, pwr_cnt);
	isp->sw_contex.power_state = ATOM_ISP_POWER_DOWN;

	return 0;
}

/*Turn on ISP dphy */
int atomisp_ospm_dphy_up(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;
	v4l2_dbg(3, dbg_level, &atomisp_dev, "%s\n", __func__);

	/* MRFLD IUNIT PHY is located in an always-power-on island */
	if (IS_MRFLD)
		return 0;

	/* power on DPHY */
	pwr_cnt = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL);
	pwr_cnt &= ~0x300;
	intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL, pwr_cnt);

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
		v4l2_warn(&atomisp_dev, "failed to g_ctrl for focal length\n");
	else
		config->focal_length = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_ABSOLUTE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev, "failed to g_ctrl for f-number\n");
	else
		config->f_number_curr = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_RANGE;
	if (v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev,
				"failed to g_ctrl for f number range\n");
	else
		config->f_number_range = ctrl.value;

	return 0;
}

int atomisp_flash_enable(struct atomisp_device *isp, int num_frames)
{
	if (num_frames < 0) {
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "%s ERROR: num_frames: %d\n", __func__, num_frames);
		return -EINVAL;
	}
	/* a requested flash is still in progress. */
	if (num_frames && isp->params.flash_state != ATOMISP_FLASH_IDLE) {
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s flash busy: %d frames left: %d\n",__func__,
				isp->params.flash_state,
				isp->params.num_flash_frames);
		return -EBUSY;
	}

	isp->params.num_flash_frames = num_frames;
	isp->params.flash_state = ATOMISP_FLASH_REQUESTED;
	return 0;
}
