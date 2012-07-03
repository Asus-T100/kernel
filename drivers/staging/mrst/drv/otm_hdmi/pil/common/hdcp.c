/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2011 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:

  Intel Corporation
  2200 Mission College Blvd.
  Santa Clara, CA  95054

  BSD LICENSE

  Copyright(c) 2011 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/types.h>
#include "hdcp_rx_defs.h"
#include "otm_hdmi.h"
#include "otm_hdmi_types.h"
#include "ipil_hdcp_api.h"

#define OTM_HDCP_DEBUG_MODULE

#ifdef OTM_HDCP_DEBUG_MODULE
bool module_disable_hdcp = false;
EXPORT_SYMBOL_GPL(module_disable_hdcp);
bool module_force_ri_mismatch = false;
EXPORT_SYMBOL_GPL(module_force_ri_mismatch);
#endif

enum {
	HDCP_ENABLE = 1,
	HDCP_RESET,
	HDCP_RI_CHECK,
	HDCP_REPEATER_CHECK,
	HDCP_REPEATER_WDT_EXPIRED,
	HDCP_SET_POWER_SAVE_STATUS,
	HDCP_SET_HPD_STATUS,
	HDCP_SET_DPMS_STATUS
} hdcp_task_msg_en;

struct hdcp_wq_struct_t {
	struct delayed_work dwork;
	int msg;
	void *msg_data;
};

/* = = = = = = = = = = = = = = = = == = = = = = = = = = = = = = = = = = = = = */
/*!  \brief Our local context.
 */
struct hdcp_context_t {
	int		can_authenticate;
			/*!< indicates HDCP Authentication currently allowed
			 */
	bool	is_required;
	bool	is_phase1_enabled;
	bool	is_phase2_enabled;
	bool	is_phase3_valid;
	bool	suspend;
	bool	hpd;
	bool	display_power_on;
	bool	auto_retry;
	bool	wdt_expired;
	bool	sink_query /* used to unconditionally read sink bksv */;
	bool	force_reset;
	unsigned int	auth_count;
			/*!< A counter that indicates current
			 * authentication attempt
			 */
	unsigned int	ri_check_interval;
			/*!< phase 3 ri-check interval based on mode */
	unsigned int	wdt_id;
			/*!< phase 2 auth watchdog timer id */
	unsigned int	current_srm_ver;
			/*!< currently used SRM version (if vrl is not null)
			 */
	uint64_t	*vrl;
			/*!< pointer to our VRL, formatted as array of KSVs
			 * as hdcp__u64_t's
			 */
	unsigned int	vrl_count;
			/*!< total number of KSVs in our VRL */
	int (*ddc_read_write)(bool, uint8_t, uint8_t, uint8_t *, int);
			/*!< Pointer to callback function for DDC Read Write */
	struct workqueue_struct *hdcp_wq;
};

/* Global instance of local context */
static struct hdcp_context_t *hdcp_context;

/* HDCP Main Event Handler */
static void hdcp_task_event_handler(struct work_struct *work);

static bool wq_send_message_delayed(int msg,
					void *msg_data,
					unsigned long delay)
{
	struct hdcp_wq_struct_t *hwq = NULL;
	hwq = kmalloc(sizeof(struct hdcp_wq_struct_t), GFP_KERNEL);
	if (hwq == NULL) {
		if (msg_data)
			kfree(msg_data);
		return false;
	}
	hwq->msg = msg;
	hwq->msg_data = msg_data;

	INIT_DELAYED_WORK(&hwq->dwork, hdcp_task_event_handler);

	if (queue_delayed_work(hdcp_context->hdcp_wq, &hwq->dwork,
		(unsigned long)(msecs_to_jiffies(delay))) != 0)
		return true;
	else
		pr_debug("hdcp: failed to add message to delayed wq\n");

	if (msg_data)
		kfree(msg_data);

	return false;
}

static bool wq_send_message(int msg, void *msg_data)
{
	return wq_send_message_delayed(msg, msg_data, 0);
}

static bool hdcp_enable_condition_ready(void)
{
	if (hdcp_context != NULL &&
	    hdcp_context->can_authenticate == true &&
	    hdcp_context->is_required == true &&
	    hdcp_context->suspend == false &&
	    hdcp_context->hpd == true &&
	    hdcp_context->display_power_on == true)
		return true;

	return false;
}

static int hdcp_ddc_read(uint8_t offset, uint8_t *buffer, int size)
{
	if (hdcp_enable_condition_ready() == true ||
		(hdcp_context->sink_query == true && 
		 offset == HDCP_RX_BKSV_ADDR))
		return hdcp_context->ddc_read_write(true,
			HDCP_PRIMARY_I2C_ADDR, offset, buffer, size);
	return false;
}

static int hdcp_ddc_write(uint8_t offset, uint8_t *buffer, int size)
{
	if (hdcp_enable_condition_ready() == true)
		return hdcp_context->ddc_read_write(false,
			HDCP_PRIMARY_I2C_ADDR, offset, buffer, size);
	return false;
}

static uint64_t hdcp_ksv_64val_conv(uint8_t *ksv, uint32_t size)
{
	int i = 0;
	uint64_t ksv64 = 0;
	if (ksv != NULL && size == HDCP_KSV_SIZE) {
		for (i = 0; i < 5; i++)
			ksv64 |= (ksv[i] << (i*8));
	}
	return ksv64;
}

static bool hdcp_validate_ksv(uint8_t *ksv, uint32_t size)
{
	int i = 0, count = 0;
	uint8_t temp = 0;
	uint64_t ksv64 = hdcp_ksv_64val_conv(ksv, size);
	bool ret = false;
	if (ksv != NULL  && size == HDCP_KSV_SIZE) {
		count = 0;
		for (i = 0; i < 5; i++) {
			temp = ksv[i];
			while (temp) {
				temp &= (temp-1);
				count++;
			}
		}
		if (count == HDCP_KSV_HAMMING_WT)
			ret = true;
	}
	if (ret) {
		/* SRM Check ? */
		if (hdcp_context->vrl != NULL) {
			const uint64_t *vrl = hdcp_context->vrl;
			for (i = 0; i < hdcp_context->vrl_count; i++, vrl++) {
				if (ksv64 == *vrl)
					return true;
			}
		}
	}
	return ret;
}

static bool hdcp_get_aksv(uint8_t *aksv, uint32_t size)
{
	bool ret = false;
	if (ipil_hdcp_get_aksv(aksv, HDCP_KSV_SIZE) == true) {
		if (hdcp_validate_ksv(aksv, size) == true)
			ret = true;
	}
	return ret;
}

static bool hdcp_read_bksv(uint8_t *bksv, uint32_t size)
{
	bool ret = false;
	if (bksv != NULL  && size == HDCP_KSV_SIZE) {
		if (hdcp_ddc_read(HDCP_RX_BKSV_ADDR,
				bksv, HDCP_KSV_SIZE) == true) {
			if (hdcp_validate_ksv(bksv, size) == true)
				ret = true;
		}
	}
	return ret;
}

static bool hdcp_read_bcaps(uint8_t *bcaps)
{
	bool ret = false;
	if (bcaps != NULL) {
		if (hdcp_ddc_read(HDCP_RX_BCAPS_ADDR,
				bcaps, HDCP_RX_BCAPS_SIZE) == true)
			ret = true;
	}
	return ret;
}

static bool hdcp_read_bstatus(uint16_t *bstatus)
{
	bool ret = false;
	if (bstatus != NULL) {
		if (hdcp_ddc_read(HDCP_RX_BSTATUS_ADDR,
			(uint8_t *)bstatus, HDCP_RX_BSTATUS_SIZE) == true)
			ret = true;
	}
	return ret;
}

static bool hdcp_read_rx_ri(uint16_t *rx_ri)
{
	bool ret = false;
	if (rx_ri != NULL) {
		if (hdcp_ddc_read(HDCP_RX_RI_ADDR,
				(uint8_t *)rx_ri, HDCP_RI_SIZE) == true)
			ret = true;
	}
	return ret;
}

static bool hdcp_read_rx_r0(uint16_t *rx_r0)
{
	return hdcp_read_rx_ri(rx_r0);
}

static bool hdcp_read_rx_ksv_list(uint8_t *ksv_list, uint32_t size)
{
	bool ret = false;
	if (ksv_list != NULL && size) {
		if (hdcp_ddc_read(HDCP_RX_KSV_FIFO_ADDR,
		    ksv_list, size) == true) {
			ret = true;
		}
	}
	return ret;
}

static bool hdcp_read_rx_v(uint8_t *v)
{
	bool ret = false;
	uint8_t *buf = v;
	uint8_t offset = HDCP_RX_V_H0_ADDR;

	if (v != NULL) {
		for (; offset <= HDCP_RX_V_H4_ADDR; offset += 4) {
			if (hdcp_ddc_read(offset, buf, 4) == false) {
				pr_debug("hdcp: read rx v failure\n");
				break;
			}
			buf += 4;
		}
		if (offset > HDCP_RX_V_H4_ADDR)
			ret = true;
	}
	return ret;
}

static bool hdcp_stage3_ri_check(void)
{
	uint16_t rx_ri = 0;

	if (hdcp_enable_condition_ready() == false ||
	    hdcp_context->is_phase1_enabled == false)
		return false;

#ifdef OTM_HDCP_DEBUG_MODULE
	if (module_force_ri_mismatch) {
		pr_debug("hdcp: force Ri mismatch\n");
		module_force_ri_mismatch = false;
		return false;
	}
#endif

	if (hdcp_read_rx_ri(&rx_ri) == true)
		if (ipil_hdcp_does_ri_match(rx_ri) == true)
			/* pr_debug("hdcp: Ri Matches %04x\n", rx_ri);*/
			return true;

	/* ri check failed update phase3 status */
	hdcp_context->is_phase3_valid = false;

	pr_debug("hdcp: error!!!  Ri check failed %x\n", rx_ri);
	return false;
}

static bool hdcp_send_an_aksv(uint8_t *an, uint8_t an_size,
			uint8_t *aksv, uint8_t aksv_size)
{
	bool ret = false;
	if (an != NULL && an_size == HDCP_AN_SIZE &&
	   aksv != NULL  && aksv_size == HDCP_KSV_SIZE) {
		if (hdcp_ddc_write(HDCP_RX_AN_ADDR, an, HDCP_AN_SIZE) ==
			true) {
			/* wait 20ms for i2c write for An to complete */
			msleep(20);
			if (hdcp_ddc_write(HDCP_RX_AKSV_ADDR, aksv,
					HDCP_KSV_SIZE) == true)
				ret = true;
		}
	}
	return ret;
}

static void hdcp_reset(void)
{
	pr_debug("hdcp: reset\n");

	/* Stop HDCP */
	if (hdcp_context->is_phase1_enabled == true ||
	    hdcp_context->force_reset == true) {
		pr_debug("hdcp: off state\n");
		ipil_hdcp_disable();
		hdcp_context->force_reset = false;
	}

#ifndef OTM_HDMI_HDCP_ALWAYS_ENC
	/* this flag will be re-enabled by upper layers */
	hdcp_context->is_required = false;
#endif
	hdcp_context->is_phase1_enabled = false;
	hdcp_context->is_phase2_enabled = false;
	hdcp_context->is_phase3_valid = false;
	hdcp_context->wdt_id++;
}

static bool hdcp_rep_check(void)
{
	int msg = HDCP_REPEATER_CHECK;
	return wq_send_message(msg, NULL);
}

static bool hdcp_rep_watch_dog(void)
{
	int msg = HDCP_REPEATER_WDT_EXPIRED;
	unsigned int *wdt_id = kmalloc(sizeof(unsigned int), GFP_KERNEL);
	if (wdt_id != NULL) {
		*wdt_id = hdcp_context->wdt_id;
		return wq_send_message_delayed(msg, (void *)wdt_id, 5000);
	} else
		pr_debug("hdcp: %s failed to alloc mem\n", __func__);
	return false;
}

static bool hdcp_initiate_rep_auth(void)
{
	pr_debug("hdcp: initiating repeater check\n");
	hdcp_context->wdt_expired = false;
	if (hdcp_rep_check() == true) {
		if (hdcp_rep_watch_dog() == true)
			return true;
		else
			pr_debug("hdcp: failed to start repeater wdt\n");
	} else
		pr_debug("hdcp: failed to start repeater check\n");
	return false;
}

static bool hdcp_stage3_schedule_ri_check(void)
{
	int msg = HDCP_RI_CHECK;
	unsigned int *msg_data = kmalloc(sizeof(unsigned int), GFP_KERNEL);
	if (msg_data != NULL) {
		*msg_data = hdcp_context->auth_count;
		return wq_send_message_delayed(msg, (void *)msg_data,
				hdcp_context->ri_check_interval);
	}
	return false;
}

static int hdcp_stage1_authentication(bool *is_repeater)
{
	uint8_t bksv[HDCP_KSV_SIZE], aksv[HDCP_KSV_SIZE], an[HDCP_AN_SIZE];
	struct hdcp_rx_bstatus_t bstatus;
	struct hdcp_rx_bcaps_t bcaps;
	uint8_t retry = 0;
	uint16_t rx_r0 = 0;

	/* Read BKSV */
	if (hdcp_read_bksv(bksv, HDCP_KSV_SIZE) == false)
		return false;
	pr_debug("hdcp: bksv: %02x%02x%02x%02x%02x\n",
		bksv[0], bksv[1], bksv[2], bksv[3], bksv[4]);

	/* Read An AKSV */
	if (ipil_hdcp_get_an(an, HDCP_AN_SIZE) == false)
		return false;
	pr_debug("hdcp: an: %02x%02x%02x%02x%02x%02x%02x%02x\n",
		an[0], an[1], an[2], an[3], an[4], an[5], an[6], an[7]);

	if (hdcp_get_aksv(aksv, HDCP_KSV_SIZE) == false)
		return false;
	pr_debug("hdcp: aksv: %02x%02x%02x%02x%02x\n",
			aksv[0], aksv[1], aksv[2], aksv[3], aksv[4]);

	/* Write An AKSV to Downstream Rx */
	if (hdcp_send_an_aksv(an, HDCP_AN_SIZE, aksv, HDCP_KSV_SIZE)
						== false)
		return false;
	pr_debug("hdcp: sent an aksv\n");

	/* Read BCAPS & BKSV */
	if (hdcp_read_bksv(bksv, HDCP_KSV_SIZE) == false)
		return false;
	pr_debug("hdcp: bksv: %02x%02x%02x%02x%02x\n",
			bksv[0], bksv[1], bksv[2], bksv[3], bksv[4]);

	/* wait 20ms for i2c read for bksv to complete */
	msleep(20);

	if (hdcp_read_bcaps(&bcaps.value) == false)
		return false;
	pr_debug("hdcp: bcaps: %x\n", bcaps.value);

	/* wait 20ms for i2c read for bcaps to complete */
	msleep(20);

	/* Read BSTATUS */
	if (hdcp_read_bstatus(&bstatus.value) == false)
		return false;
	pr_debug("hdcp: bstatus: %04x\n", bstatus.value);

	/* Update repeater present status */
	*is_repeater = bcaps.is_repeater;

	/* Set Repeater Bit */
	if (ipil_hdcp_set_repeater(bcaps.is_repeater) == false)
		return false;

	/* Write BKSV to Self */
	if (ipil_hdcp_set_bksv(bksv) == false)
		return false;

	pr_debug("hdcp: set repeater & bksv\n");

	/* Start First Stage of Authenticatioin */
	if (ipil_hdcp_start_authentication() == false)
		return false;

	pr_debug("hdcp: auth started\n");

	/* Wait for 120ms before reading R0' */
	msleep(120);

	/* Check if R0 Ready */
	retry = 20;
	do {
		if (ipil_hdcp_is_r0_ready() == true)
			break;
		msleep(5);
		retry--;
	} while (retry);

	if (retry == 0 && ipil_hdcp_is_r0_ready() == false)
		return false;

	pr_debug("hdcp: R0 ready\n");

	/* Read Ro' from Receiver */
	if (hdcp_read_rx_r0(&rx_r0) == false)
		return false;
	pr_debug("hdcp: rx_r0 = %04x\n", rx_r0);

	/* Check if R0 Matches */
	if (ipil_hdcp_does_ri_match(rx_r0) == false)
		return false;
	pr_debug("hdcp: R0 matched\n");

	/* Enable Encryption & Check status */
	if (ipil_hdcp_enable_encryption() == false)
		return false;
	pr_debug("hdcp: encryption enabled\n");

	hdcp_context->is_phase1_enabled = true;

	return true;
}

static int hdcp_stage2_repeater_authentication(void)
{
	uint8_t *rep_ksv_list = NULL;
	uint32_t rep_prime_v[HDCP_V_H_SIZE] = {0};
	struct hdcp_rx_bstatus_t bstatus;
	struct hdcp_rx_bcaps_t bcaps;
	bool ret = false;

	/* Repeater Authentication */
	if (hdcp_enable_condition_ready() == false ||
	    hdcp_context->is_phase1_enabled == false ||
	    hdcp_context->wdt_expired == true) {
		pr_debug("hdcp: stage2 auth condition not ready\n");
		return false;
	}

	/* Read BCAPS */
	if (hdcp_read_bcaps(&bcaps.value) == false)
		return false;

	if (!bcaps.is_repeater)
		return false;

	/* Check if fifo ready */
	if (!bcaps.ksv_fifo_ready) {
		/* reschedule if not ready */
		pr_debug("hdcp: rescheduling repeater auth\n");
		msleep(100);
		hdcp_rep_check();
		return true;
	}

	/* Read BSTATUS */
	if (hdcp_read_bstatus(&bstatus.value) == false)
		return false;

	/* Check validity of repeater depth & device count */
	if (bstatus.max_devs_exceeded)
		return false;

	if (bstatus.max_cascade_exceeded)
		return false;

	if (0 == bstatus.device_count)
		return true;

	if (bstatus.device_count > HDCP_MAX_DEVICES)
		return false;

	/* Set repeater bit */
	if (ipil_hdcp_set_repeater(bcaps.is_repeater) == false)
		return false;

	/* Read ksv list from repeater */
	rep_ksv_list = kzalloc(bstatus.device_count * HDCP_KSV_SIZE, GFP_KERNEL);
	if (!rep_ksv_list) {
		pr_debug("hdcp: rep ksv list alloc failure\n");
		return false;
	}

	if (hdcp_read_rx_ksv_list(rep_ksv_list,
				  bstatus.device_count * HDCP_KSV_SIZE)
				  == false) {
		pr_debug("hdcp: rep ksv list read failure\n");
		goto exit;
	}

	/* TODO: SRM check */

	/* Compute tx(v) */
	if (ipil_hdcp_compute_tx_v(rep_ksv_list, bstatus.device_count,
				   bstatus.value) == false) {
		pr_debug("hdcp: rep compute tx v failure\n");
		goto exit;
	}

	/* Read rx(v') */
	if (hdcp_read_rx_v((uint8_t *)rep_prime_v) == false) {
		pr_debug("hdcp: rep read rx v failure\n");
		ipil_hdcp_set_repeater(0);
		goto exit;
	}

	/* Verify SHA1 tx(v) = rx(v') */
	if (ipil_hdcp_compare_v(rep_prime_v) == false) {
		pr_debug("hdcp: rep compare v failure\n");
		goto exit;
	}

	pr_debug("hdcp: repeater auth success\n");
	hdcp_context->is_phase2_enabled = true;
	ret = true;

exit:
	kfree(rep_ksv_list);
	return ret;
}

static bool hdcp_start(void)
{
	bool is_repeater = false;

	pr_debug("hdcp: start\n");

	/* Increment Auth Check Counter */
	hdcp_context->auth_count++;

	/* Check HDCP Status */
	if (ipil_hdcp_is_ready() == false)
		return false;

	if (hdcp_stage1_authentication(&is_repeater) == false)
		return false;

	pr_debug("hdcp: initial authentication completed, repeater:%d\n",
		is_repeater);

	/* Branch Repeater Mode Authentication */
	if (is_repeater == true)
		if (hdcp_initiate_rep_auth() == false)
			return false;

	/* Initiate phase3_valid with true status */
	hdcp_context->is_phase3_valid = true;
	/* Branch Periodic Ri Check */
	pr_debug("hdcp: starting periodic Ri check\n");
	if (hdcp_stage3_schedule_ri_check() == false)
		return false;

	return true;
}

static void hdcp_retry_enable(void)
{
	int msg = HDCP_ENABLE;
	if (hdcp_enable_condition_ready() == true &&
		hdcp_context->is_phase1_enabled == false &&
		hdcp_context->auto_retry == true) {
		wq_send_message_delayed(msg, NULL, 30);
		pr_debug("hdcp: retry enable\n");
	}
}

static void hdcp_task_event_handler(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct hdcp_wq_struct_t *hwq = container_of(delayed_work,
						struct hdcp_wq_struct_t,
						dwork);
	int msg = 0;
	void *msg_data = NULL;
	bool reset_hdcp = false;

	if (hwq != NULL) {
		msg = hwq->msg;
		msg_data = hwq->msg_data;
	}

	if (hdcp_context == NULL || hwq == NULL)
		goto EXIT_HDCP_HANDLER;

	switch (msg) {
	case HDCP_ENABLE:
#ifndef OTM_HDMI_HDCP_ALWAYS_ENC
		if (hdcp_enable_condition_ready() == false) {
			reset_hdcp = true;
			break;
		}
#endif
		if (hdcp_enable_condition_ready() == true &&
		    hdcp_context->is_phase1_enabled == false &&
		    hdcp_start() == false) {
			reset_hdcp = true;
			hdcp_context->force_reset = true;
			pr_debug("hdcp: failed to start hdcp\n");
		}
		break;

	case HDCP_RI_CHECK:
		/*pr_debug("hdcp: RI CHECK\n");*/
		if (msg_data == NULL ||
		    *(unsigned int *)msg_data != hdcp_context->auth_count)
			/*pr_debug("hdcp: auth count %d mismatch %d\n",
				*(unsigned int *)msg_data,
				hdcp_context->auth_count);*/
			break;

		if (hdcp_stage3_ri_check() == false ||
		    hdcp_stage3_schedule_ri_check() == false)
			reset_hdcp = true;
		break;

	case HDCP_REPEATER_CHECK:
		pr_debug("hdcp: repeater check\n");
		if (hdcp_stage2_repeater_authentication() == false)
			reset_hdcp = true;
		break;

	case HDCP_REPEATER_WDT_EXPIRED:
		if (msg_data != NULL) {
			pr_debug("hdcp: reapter wdt expired, "
				    "wdt_id = %d, msg_data = %d\n",
				    hdcp_context->wdt_id,
				    *(unsigned int *)msg_data);
			hdcp_context->wdt_expired = true;
			if (hdcp_context->wdt_id == *((unsigned int *)msg_data)
			    && !hdcp_context->is_phase2_enabled)
				reset_hdcp = true;
		}
		break;

	case HDCP_RESET:
		hdcp_reset();
		break;

	case HDCP_SET_POWER_SAVE_STATUS:/* handle suspend resume */
		/* ignore suspend state if HPD is low */
		if (msg_data != NULL && hdcp_context->hpd == true) {
			hdcp_context->suspend = *((bool *)msg_data);
			pr_debug("hdcp: suspend = %d\n",
					hdcp_context->suspend);
			if (hdcp_context->suspend == true) {
				if (hdcp_context->is_phase1_enabled
				    == true)
					reset_hdcp = true;
			}
		}
		break;

	case HDCP_SET_HPD_STATUS:/* handle hpd status */
		if (msg_data != NULL) {
			hdcp_context->hpd = *((bool *)msg_data);
			pr_debug("hdcp: hpd = %d\n",
					hdcp_context->hpd);
			if (hdcp_context->hpd == false) {
				/* reset suspend state if HPD is Low */
				hdcp_context->suspend = false;
				reset_hdcp = true;
			}
		}
		break;

	case HDCP_SET_DPMS_STATUS:/* handle display_power_on status */
		if (msg_data != NULL) {
			hdcp_context->display_power_on =
					*((bool *)msg_data);
			pr_debug("hdcp: display_power_on = %d\n",
					hdcp_context->display_power_on);
			if (hdcp_context->display_power_on == false)
				reset_hdcp = true;
		}
		break;

	default:
		break;
	}

	if (reset_hdcp == true) {
		msg = HDCP_RESET;
		wq_send_message(msg, NULL);
	} else
		/* if disabled retry HDCP authentication */
		hdcp_retry_enable();
EXIT_HDCP_HANDLER:
	if (msg_data != NULL)
		kfree(msg_data);
	if (hwq != NULL)
		kfree(hwq);

	return;
}

/*
 * Description: function to update HPD status
 *
 * @hpdi	HPD high/low status
 *
 * Returns:	true on success
 *		false on failure
 */
bool otm_hdmi_hdcp_set_hpd_state(hdmi_context_t *hdmi_context,
					bool hpd)
{
	int msg = HDCP_SET_HPD_STATUS;
	bool *p_hpd = NULL;
	hpd = !!(hpd);

	if (hdmi_context == NULL || hdcp_context == NULL)
		return false;

	if (hdcp_context->hpd != hpd) {
		p_hpd = kmalloc(sizeof(bool), GFP_KERNEL);
		if (p_hpd != NULL) {
			*p_hpd = hpd;
			return wq_send_message(msg, (void *)p_hpd);
		} else
			pr_debug("hdcp: %s failed to alloc mem\n", __func__);
	}

	return false;
}

/*
 * Description: function to update power save (suspend/resume) status
 *
 * @suspend	suspend/resume status
 *
 * Returns:	true on success
 *		false on failure
 */
bool otm_hdmi_hdcp_set_power_save(hdmi_context_t *hdmi_context,
					bool suspend)
{
	int msg = HDCP_SET_POWER_SAVE_STATUS;
	bool *p_suspend = NULL;
	suspend = !!(suspend);

	if (hdmi_context == NULL || hdcp_context == NULL)
		return false;

	if (hdcp_context->suspend != suspend) {
		p_suspend = kmalloc(sizeof(bool), GFP_KERNEL);
		if (p_suspend != NULL) {
			*p_suspend = suspend;
			return wq_send_message(msg, (void *)p_suspend);
		} else
			pr_debug("hdcp: %s failed to alloc mem\n", __func__);
		if (suspend == true)
			/* Cleanup WorkQueue */
			/* TODO: Needs Cosai Calls */
			flush_workqueue(hdcp_context->hdcp_wq);
	}

	return false;
}

/*
 * Description: function to update display_power_on status
 *
 * @display_power_on	display power on/off status
 *
 * Returns:	true on success
 *		false on failure
 */
bool otm_hdmi_hdcp_set_dpms(hdmi_context_t *hdmi_context,
			bool display_power_on)
{
#ifdef OTM_HDMI_HDCP_ALWAYS_ENC
	int msg = HDCP_SET_DPMS_STATUS;
	bool *p_display_power_on = NULL;
	display_power_on = !!(display_power_on);
#endif

	if (hdmi_context == NULL || hdcp_context == NULL)
		return false;

#ifndef OTM_HDMI_HDCP_ALWAYS_ENC
	return true;
#else
	if (hdcp_context->display_power_on != display_power_on) {
		p_display_power_on = kmalloc(sizeof(bool), GFP_KERNEL);
		if (p_display_power_on != NULL) {
			*p_display_power_on = display_power_on;
			return wq_send_message(msg, (void *)p_display_power_on);
		} else
			pr_debug("hdcp: %s failed to alloc mem\n", __func__);
		if (display_power_on == false)
			/* Cleanup WorkQueue */

			flush_workqueue(hdcp_context->hdcp_wq);
	}
	return false;
#endif
}

/*
 * Description: Function to check HDCP encryption status
 *
 * Returns:	true if encrypting
 *		else false
 */
bool otm_hdmi_hdcp_enc_status(hdmi_context_t *hdmi_context)
{
#ifndef OTM_HDMI_HDCP_ALWAYS_ENC
	if (hdmi_context == NULL || hdcp_context == NULL)
		return false;

	if (hdcp_context->is_required && hdcp_context->is_phase1_enabled)
		return true;
#endif
	return false;	
}

/*
 * Description: Function to check HDCP Phase3 Link status
 *
 * Returns:	true if link is verified Ri Matches
 *		else false
 */
bool otm_hdmi_hdcp_link_status(hdmi_context_t *hdmi_context)
{
#ifndef OTM_HDMI_HDCP_ALWAYS_ENC
	if (hdmi_context == NULL || hdcp_context == NULL)
		return false;

	if (hdcp_context->is_phase3_valid)
		return true;
#endif
	return false;	
}

/*
 * Description: Function to read BKSV and validate
 *
 * Returns:	true on success
 *		false on failure
 */
bool otm_hdmi_hdcp_read_validate_bksv(hdmi_context_t *hdmi_context,
				uint8_t *bksv)
{
	bool ret = false;
#ifndef OTM_HDMI_HDCP_ALWAYS_ENC
	if (hdmi_context == NULL || hdcp_context == NULL || bksv == NULL)
		return false;

	if(hdcp_context->hpd ==true &&
		hdcp_context->suspend == false &&
		hdcp_context->display_power_on == true &&
		hdcp_context->is_required == false &&
		hdcp_context->is_phase1_enabled == false) {
		hdcp_context->sink_query = true;
		ret = hdcp_read_bksv(bksv, HDCP_KSV_SIZE);
		hdcp_context->sink_query = false;
	}
#endif
	return ret;
}

/*
 * Description: function to enable HDCP
 *
 * Returns:	true on success
 *		false on failure
 */
bool otm_hdmi_hdcp_enable(hdmi_context_t *hdmi_context,
				int refresh_rate)
{
	int msg = HDCP_ENABLE;

	if (hdmi_context == NULL || hdcp_context == NULL)
		return false;

	if (hdcp_context->is_required == true) {
		pr_debug("hdcp: already enabled\n");
		return true;
	}
#ifdef OTM_HDCP_DEBUG_MODULE
	if (module_disable_hdcp) {
		pr_debug("hdcp: disabled by module\n");
		return false;
	}
#endif

	hdcp_context->is_required = true;
	if (refresh_rate)
		/* compute msec time for 128 frames per HDCP spec */
		hdcp_context->ri_check_interval = ((128 * 1000) / refresh_rate);
	else
		/* default to 128 frames @ 60 Hz */
		hdcp_context->ri_check_interval = ((128 * 1000) / 60);

	pr_debug("hdcp: enable\n");

#ifdef OTM_HDMI_HDCP_ALWAYS_ENC
	return wq_send_message(msg, NULL);
#else
	/* send message and wait for 1st stage authentication to complete */
	if (wq_send_message(msg, NULL)) {
		while (hdcp_context->is_required) {
			if(hdcp_context->is_phase1_enabled)
				return true;
			msleep(1);
		}
	}

	return false;
#endif
}

/*
 * Description: function to disable HDCP
 *
 * Returns:	true on success
 *		false on failure
 */
bool otm_hdmi_hdcp_disable(hdmi_context_t *hdmi_context)
{
	int msg = HDCP_RESET;

	if (hdmi_context == NULL || hdcp_context == NULL)
		return false;

	if (hdcp_context->is_required == false) {
		pr_debug("hdcp: already disabled\n");
		return true;
	}

	wq_send_message(msg, NULL);

	/* Cleanup WorkQueue */
	flush_workqueue(hdcp_context->hdcp_wq);

	hdcp_context->is_required = false;

	pr_debug("hdcp: disable\n");

	return true;
}

/*
 * Description: hdcp init function
 *
 * @ddc_rd_wr:	pointer to ddc read write function
 *
 * Returns:	true on success
 *		false on failure
 */
bool otm_hdmi_hdcp_init(hdmi_context_t *hdmi_context,
	int (*ddc_rd_wr)(bool, uint8_t, uint8_t, uint8_t *, int))
{
	if (hdmi_context == NULL ||
	    ddc_rd_wr == NULL ||
	    ipil_hdcp_device_can_authenticate() == false ||
	    hdcp_context != NULL) {
		pr_debug("hdcp: init error!!! parameters\n");
		return false;
	}

	hdcp_context = kmalloc(sizeof(struct hdcp_context_t), GFP_KERNEL);

	if (hdcp_context != NULL)
		hdcp_context->hdcp_wq = create_workqueue("HDCP_WQ");

	if (hdcp_context == NULL || hdcp_context->hdcp_wq == NULL) {
		pr_debug("hdcp: init error!!! allocation\n");
		goto EXIT_INIT;
	}

	hdcp_context->is_required	= false;
	hdcp_context->is_phase1_enabled	= false;
	hdcp_context->is_phase2_enabled	= false;
	hdcp_context->is_phase3_valid	= false;
	hdcp_context->suspend		= false;
	hdcp_context->hpd		= false;
#ifndef OTM_HDMI_HDCP_ALWAYS_ENC
	hdcp_context->display_power_on	= true;
	hdcp_context->auto_retry	= false;
#else
	hdcp_context->display_power_on	= false;
	hdcp_context->auto_retry	= true;
#endif
	hdcp_context->wdt_expired	= false;
	hdcp_context->sink_query	= false;
	hdcp_context->can_authenticate	= true;
	hdcp_context->wdt_id		= 1u;
	hdcp_context->current_srm_ver	= 0u;
	hdcp_context->vrl		= NULL;
	hdcp_context->vrl_count		= 0u;
	hdcp_context->ri_check_interval	= 0u;
	hdcp_context->force_reset	= false;
	hdcp_context->auth_count	= 0;

	hdcp_context->ddc_read_write = ddc_rd_wr;

	if (ipil_hdcp_init() == true) {
		pr_debug("hdcp: initialized\n");
		return true;
	}
EXIT_INIT:
	/* Cleanup and exit */
	if (hdcp_context != NULL) {
		if (hdcp_context->hdcp_wq != NULL)
			destroy_workqueue(hdcp_context->hdcp_wq);
		kfree(hdcp_context);
		hdcp_context = NULL;
	}

	return false;
}

