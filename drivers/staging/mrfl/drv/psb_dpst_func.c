/******************************************************************************
 *
 * Copyright (c) 2011, Intel Corporation.
 * Portions (c), Imagination Technology, Ltd.
 * All rights reserved.
 *
 * Redistribution and Use.  Redistribution and use in binary form, without
 * modification, of the software code provided with this license ("Software"),
 * are permitted provided that the following conditions are met:
 *
 *  1. Redistributions must reproduce the above copyright notice and this
 *     license in the documentation and/or other materials provided with the
 *     Software.
 *  2. Neither the name of Intel Corporation nor the name of Imagination
 *     Technology, Ltd may be used to endorse or promote products derived from
 *     the Software without specific prior written permission.
 *  3. The Software can only be used in connection with the Intel hardware
 *     designed to use the Software as outlined in the documentation. No other
 *     use is authorized.
 *  4. No reverse engineering, decompilation, or disassembly of the Software
 *     is permitted.
 *  5. The Software may not be distributed under terms different than this
 *     license.
 *
 * Limited Patent License.  Intel Corporation grants a world-wide, royalty-free
 * , non-exclusive license under patents it now or hereafter owns or controls
 * to make, have made, use, import, offer to sell and sell ("Utilize") the
 * Software, but solely to the extent that any such patent is necessary to
 * Utilize the Software alone.  The patent license shall not apply to any
 * combinations which include the Software.  No hardware per se is licensed
 * hereunder.
 *
 * Ownership of Software and Copyrights. Title to all copies of the Software
 * remains with the copyright holders. The Software is copyrighted and
 * protected by the laws of the United States and other countries, and
 * international treaty provisions.
 *
 * DISCLAIMER.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <linux/version.h>

#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_dpst.h"
#include "dispmgrnl.h"
#include "psb_dpst_func.h"

static struct drm_device *g_dev;   /* hack for the queue */

int send_hist()
{
	struct dispmgr_command_hdr dispmgr_cmd;
	struct drm_psb_hist_status_arg mydata;

	/* before we send get the status for run_algorithm */
	dpst_histogram_get_status(g_dev, &mydata);

	dispmgr_cmd.module = DISPMGR_MOD_DPST;
	dispmgr_cmd.cmd = DISPMGR_DPST_HIST_DATA;
	dispmgr_cmd.data_size = sizeof(struct drm_psb_hist_status_arg);
	dispmgr_cmd.data = &mydata;
	dispmgr_nl_send_msg(&dispmgr_cmd);

	return 0;
}

/* IOCTL - moved to standard calls for Kernel Integration */

int psb_hist_enable(struct drm_device *dev, void *data)
{
	u32 irqCtrl = 0;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct dpst_guardband guardband_reg;
	struct dpst_ie_histogram_control ie_hist_cont_reg;
	uint32_t *enable = data;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		return 0;
	}

	if (*enable == 1) {
		ie_hist_cont_reg.data = PSB_RVDC32(HISTOGRAM_LOGIC_CONTROL);
		ie_hist_cont_reg.ie_pipe_assignment = 0;
		ie_hist_cont_reg.histogram_mode_select = DPST_YUV_LUMA_MODE;
		ie_hist_cont_reg.ie_histogram_enable = 1;
		PSB_WVDC32(ie_hist_cont_reg.data, HISTOGRAM_LOGIC_CONTROL);

		guardband_reg.data = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
		guardband_reg.interrupt_enable = 1;
		guardband_reg.interrupt_status = 1;
		PSB_WVDC32(guardband_reg.data, HISTOGRAM_INT_CONTROL);

		irqCtrl = PSB_RVDC32(PIPEASTAT);
		PSB_WVDC32(irqCtrl | PIPE_DPST_EVENT_ENABLE, PIPEASTAT);
		/* Wait for two vblanks */
	} else {
		guardband_reg.data = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
		guardband_reg.interrupt_enable = 0;
		guardband_reg.interrupt_status = 1;
		PSB_WVDC32(guardband_reg.data, HISTOGRAM_INT_CONTROL);

		ie_hist_cont_reg.data = PSB_RVDC32(HISTOGRAM_LOGIC_CONTROL);
		ie_hist_cont_reg.ie_histogram_enable = 0;
		PSB_WVDC32(ie_hist_cont_reg.data, HISTOGRAM_LOGIC_CONTROL);

		irqCtrl = PSB_RVDC32(PIPEASTAT);
		irqCtrl &= ~PIPE_DPST_EVENT_ENABLE;
		PSB_WVDC32(irqCtrl, PIPEASTAT);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

static int psb_hist_status(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct drm_psb_hist_status_arg *hist_status = data;
	uint32_t *arg = hist_status->buf;
	u32 iedbr_reg_data = 0;
	struct dpst_ie_histogram_control ie_hist_cont_reg;
	u32 i;
	int dpst3_bin_threshold_count = 0;
	uint32_t blm_hist_ctl = HISTOGRAM_LOGIC_CONTROL;
	uint32_t iebdr_reg = HISTOGRAM_BIN_DATA;
	uint32_t segvalue_max_22_bit = 0x3fffff;
	uint32_t iedbr_busy_bit = 0x80000000;
	int dpst3_bin_count = 32;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		return 0;
	}

	ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
	ie_hist_cont_reg.bin_reg_func_select = dpst3_bin_threshold_count;
	ie_hist_cont_reg.bin_reg_index = 0;

	PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);

	for (i = 0; i < dpst3_bin_count; i++) {
		iedbr_reg_data = PSB_RVDC32(iebdr_reg);

		if (!(iedbr_reg_data & iedbr_busy_bit)) {
			arg[i] = iedbr_reg_data & segvalue_max_22_bit;
		} else {
			i = 0;
			ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
			ie_hist_cont_reg.bin_reg_index = 0;
			PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);
		}
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

/* SH START DIET */
/* SH TODO This doesn't work yet. */
int psb_diet_enable(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	uint32_t *arg = data;
	u32 iedbr_reg_data = 0;
	struct dpst_ie_histogram_control ie_hist_cont_reg;
	u32 i;
	int dpst3_bin_threshold_count = 1;
	int dpst_hsv_multiplier = 2;
	uint32_t blm_hist_ctl = HISTOGRAM_LOGIC_CONTROL;
	uint32_t iebdr_reg = HISTOGRAM_BIN_DATA;
	uint32_t segvalue_max_22_bit = 0x3fffff;
	uint32_t iedbr_busy_bit = 0x80000000;
	int dpst3_bin_count = 32;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		return 0;
	}

	if (data) {
		ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
		ie_hist_cont_reg.bin_reg_func_select =
		    dpst3_bin_threshold_count;
		ie_hist_cont_reg.bin_reg_index = 0;

		PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);

		for (i = 0; i < dpst3_bin_count; i++)
			PSB_WVDC32(iebdr_reg, arg[i]);

		ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
		ie_hist_cont_reg.ie_mode_table_enabled = 1;
		ie_hist_cont_reg.alt_enhancement_mode = dpst_hsv_multiplier;

		PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);
	} else {
		ie_hist_cont_reg.data = PSB_RVDC32(blm_hist_ctl);
		ie_hist_cont_reg.ie_mode_table_enabled = 0;

		PSB_WVDC32(ie_hist_cont_reg.data, blm_hist_ctl);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

/* SH END */

int psb_init_comm(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct pci_dev *pdev = NULL;
	struct device *ddev = NULL;
	struct kobject *kobj = NULL;
	uint32_t *arg = data;

	if (*arg == 1) {
		/*find handle to drm kboject */
		pdev = dev->pdev;
		ddev = &pdev->dev;
		kobj = &ddev->kobj;

		if (dev_priv->psb_dpst_state == NULL) {
			/*init dpst kmum comms */
			dev_priv->psb_dpst_state = psb_dpst_init(kobj);
		}

		psb_irq_enable_dpst(dev);
		psb_dpst_notify_change_um(DPST_EVENT_INIT_COMPLETE,
					  dev_priv->psb_dpst_state);
	} else {
		/*hotplug and dpst destroy examples */
		psb_irq_disable_dpst(dev);
		psb_dpst_notify_change_um(DPST_EVENT_TERMINATE,
					  dev_priv->psb_dpst_state);
		psb_dpst_device_pool_destroy(dev_priv->psb_dpst_state);
		dev_priv->psb_dpst_state = NULL;
	}
	return 0;
}

int psb_dpst_mode(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	uint32_t *arg = data;
	uint32_t x;
	uint32_t y;
	uint32_t reg;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		return 0;
	}

	reg = PSB_RVDC32(PIPEASRC);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	/* horizontal is the left 16 bits */
	x = reg >> 16;
	/* vertical is the right 16 bits */
	y = reg & 0x0000ffff;

	/* the values are the image size minus one */
	x += 1;
	y += 1;

	*arg = (x << 16) | y;

	return 0;
}

int psb_update_guard(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct dpst_guardband *input = (struct dpst_guardband *)data;
	struct dpst_guardband reg_data;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		return 0;
	}

	reg_data.data = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
	reg_data.guardband = input->guardband;
	reg_data.guardband_interrupt_delay = input->guardband_interrupt_delay;
	/* printk(KERN_ALERT "guardband = %u\ninterrupt delay = %u\n",
	   reg_data.guardband, reg_data.guardband_interrupt_delay); */
	PSB_WVDC32(reg_data.data, HISTOGRAM_INT_CONTROL);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

/* Initialize the dpst data */
int dpst_init(struct drm_device *dev, int level, int output_id)
{
	g_dev = dev;
	/* hack for now - the work queue does not have the device */

	return 0;
}

int dpst_disable(struct drm_device *dev)
{
	uint32_t enable = 0;
	int ret = 0;

	ret = psb_init_comm(dev, &enable);

	return ret;
}

void dpst_process_event(struct umevent_obj *notify_disp_obj, int dst_group_id)
{
	struct drm_psb_private *dev_priv = g_dev->dev_private;
	int messageType;
	int do_not_quit = 1;

	/* Call into UMComm layer to receive histogram interrupts */
	/* eventval = Xpsb_kmcomm_get_kmevent((void *)tid); */
	/* fprintf(stderr, "Got message %d for DPST\n", eventval); */
	messageType = notify_disp_obj->kobj.name[0];
	/* need to debug to figure out which field this is */

	switch (messageType) {
	case 'i':		/* DPST_EVENT_INIT_COMPLETE: */
	case 'h':		/* DPST_EVENT_HIST_INTERRUPT: */
		/* DPST histogram */
		send_hist();
		break;
	case 'p':		/* DPST_EVENT_PHASE_COMPLETE: */
		break;
	case 't':		/* DPST_EVENT_TERMINATE: */
		break;
	default:
		/* disable DPST */
		do_not_quit = 0;
		break;
	}
}

int dpst_histogram_enable(struct drm_device *dev, int enable)
{
	int ret = 0;

	/* enable histogram interrupts */
	ret = psb_hist_enable(dev, &enable);
	return ret;
}

int dpst_histogram_get_status(struct drm_device *dev,
			      struct drm_psb_hist_status_arg *hist_data)
{
	int ret = 0;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	ret = psb_hist_status(dev, hist_data);
	if (ret) {
		printk(KERN_ERR
		       "Error: histogram get status ioctl returned error: %d\n",
		       ret);
		return 1;
	}
	return 0;
}

int psb_dpst_bl(struct drm_device *dev, void *data)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	uint32_t *arg = data;
	struct backlight_device bd;
	dev_priv->blc_adj2 = *arg;

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	bd.props.brightness = psb_get_brightness(&bd);
	psb_set_brightness(&bd);
#endif
	return 0;
}

int psb_gamma_set(struct drm_device *dev, void *data)
{
	uint16_t *lut_arg = data;
/*	struct drm_mode_object *obj; */
	struct drm_crtc *crtc;
	struct drm_connector *connector;
	struct psb_intel_crtc *psb_intel_crtc;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int i = 0;
/*      int32_t obj_id; */

/*      obj_id = lut_arg->output_id;
	obj = drm_mode_object_find(dev, obj_id, DRM_MODE_OBJECT_CONNECTOR);
	if (!obj) {
		printk(KERN_ERR "Invalid Connector object, id = %d\n", obj_id);
		DRM_DEBUG("Invalid Connector object.\n");
		return -EINVAL;
	}
*/

	connector = dev_priv->dpst_lvds_connector;
		/* = obj_to_connector(obj); */

	crtc = connector->encoder->crtc;
	psb_intel_crtc = to_psb_intel_crtc(crtc);

	for (i = 0; i < 256; i++)
		psb_intel_crtc->lut_adj[i] = lut_arg[i];

	psb_intel_crtc_load_lut(crtc);

	return 0;
}

void dpst_execute_recv_command(struct dispmgr_command_hdr *cmd_hdr)
{
	switch (cmd_hdr->cmd) {
	case DISPMGR_DPST_GET_MODE:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
			}

			uint32_t xy = 0;
			psb_dpst_mode(g_dev, &xy);
			struct dispmgr_command_hdr send_cmd_hdr;
			send_cmd_hdr.data_size = sizeof(xy);
			send_cmd_hdr.data = &xy;
			send_cmd_hdr.module = DISPMGR_MOD_DPST;
			send_cmd_hdr.cmd = DISPMGR_DPST_GET_MODE;
			dispmgr_nl_send_msg(&send_cmd_hdr);
		}
		break;
	case DISPMGR_DPST_INIT_COMM:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t enable = value;
				psb_init_comm(g_dev, &enable);
			}
		}
		break;
	case DISPMGR_DPST_UPDATE_GUARD:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t gb_arg = value;
				psb_update_guard(g_dev, &gb_arg);
			}
		}
		break;
	case DISPMGR_DPST_BL_CMD:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t data = value;
				psb_dpst_bl(g_dev, (void *)&data);
			}
		}
		break;
	case DISPMGR_DPST_HIST_ENABLE:
		{
			if (cmd_hdr->data_size) {
				unsigned long value =
				    *((unsigned long *)cmd_hdr->data);
				uint32_t enable = value;
				psb_hist_enable(g_dev, &enable);
			}
		}
		break;
	case DISPMGR_DPST_GAMMA_SET_CMD:
		{
			if (cmd_hdr->data_size) {
				uint16_t *arg = (uint16_t *) cmd_hdr->data;
				psb_gamma_set(g_dev, (void *)arg);
			}
		}
		break;
	case DISPMGR_DPST_DIET_ENABLE:
		{
			if (cmd_hdr->data_size) {
				uint32_t *arg = (uint32_t *) cmd_hdr->data;
				psb_diet_enable(g_dev, (void *)arg);
			}
		}
		break;
	case DISPMGR_DPST_DIET_DISABLE:
		{
			psb_diet_enable(g_dev, 0);
		}
		break;
	default:
		{
			printk
			    ("kdispmgr: received unknown dpst command = %d.\n",
			     cmd_hdr->cmd);
		};
	};			/* switch */
}
