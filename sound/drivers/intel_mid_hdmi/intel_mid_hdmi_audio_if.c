/*
 *   intel_mid_hdmi_audio_if.c - Intel HDMI audio driver for MID
 *
 *  Copyright (C) 2010 Intel Corp
 *  Authors:	Sailaja Bandarupalli <sailaja.bandarupalli@intel.com>
 *		Ramesh Babu K V <ramesh.babu@intel.com>
 *		Vaibhav Agarwal <vaibhav.agarwal@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * ALSA driver for Intel MID HDMI audio controller.  This file contains
 * interface functions exposed to HDMI Display driver and code to register
 * with ALSA framework..
 */

#define pr_fmt(fmt)		"had: " fmt

#include <linux/io.h>
#include <linux/jiffies.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include "intel_mid_hdmi_audio.h"

static int flag_en_allbufs;

/**
 * hdmi_audio_suspend - power management suspend function
 *
 *@haddata: pointer to HAD private data
 *@event: pm event for which this method is invoked
 *
 * This function is called by client driver to suspend the
 * hdmi audio.
 */
int hdmi_audio_suspend(void *haddata, pm_event_t event)
{
	int caps, retval = 0;
	struct had_pvt_data *had_stream;
	unsigned long flag_irqs;
	struct snd_intelhad *intelhaddata = (struct snd_intelhad *)haddata;

	pr_debug("Enter:%s", __func__);

	had_stream = intelhaddata->private_data;

	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	if (had_stream->stream_status > HAD_RUNNING_SILENCE) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_err("audio stream is active\n");
		return -EAGAIN;
	}

	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_debug("had not connected\n");
		return retval;
	}

	if (intelhaddata->drv_status == HAD_DRV_SUSPENDED) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_debug("had already suspended\n");
		return retval;
	}

	intelhaddata->drv_status = HAD_DRV_SUSPENDED;
	spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
	caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
	had_set_caps(HAD_SET_DISABLE_AUDIO_INT, &caps);
	had_set_caps(HAD_SET_DISABLE_AUDIO, NULL);
	retval = snd_intelhad_stop_silence(intelhaddata);
	pr_debug("Exit:%s", __func__);
	return retval;
}

/**
 * hdmi_audio_resume - power management resume function
 *
 *@haddata: pointer to HAD private data
 *
 * This function is called by client driver to resume the
 * hdmi audio.
 */
int hdmi_audio_resume(void *haddata)
{
	int caps, retval = 0;
	struct snd_intelhad *intelhaddata = (struct snd_intelhad *)haddata;
	unsigned long flag_irqs;

	pr_debug("Enter:%s", __func__);

	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_debug("had not connected\n");
		return 0;
	}

	if (HAD_DRV_SUSPENDED != intelhaddata->drv_status) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_err("had is not in suspended state\n");
		return 0;
	}

	if (had_get_hwstate(intelhaddata)) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_err("Failed to resume. Device not accessible\n");
		return -ENODEV;
	}

	intelhaddata->drv_status = HAD_DRV_CONNECTED;
	spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
	caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
	retval = had_set_caps(HAD_SET_ENABLE_AUDIO_INT, &caps);
	retval = had_set_caps(HAD_SET_ENABLE_AUDIO, NULL);
	retval = snd_intelhad_configure_silence(intelhaddata);
	retval = snd_intelhad_start_silence(intelhaddata);
	pr_debug("Exit:%s", __func__);
	return retval;
}

static inline int had_chk_intrmiss(struct snd_intelhad *intelhaddata,
		enum intel_had_aud_buf_type buf_id)
{
	int i, intr_count = 0;
	enum intel_had_aud_buf_type buff_done;
	u32 buf_size, buf_addr;
	struct had_pvt_data *had_stream;
	unsigned long flag_irqs;

	had_stream = intelhaddata->private_data;

	buff_done = buf_id;

	intr_count = snd_intelhad_read_len(intelhaddata);
	if (intr_count > 1) {
		/* In case of active playback */
		pr_err("Driver detected %d missed buffer done interrupt(s)!!!!\n",
				(intr_count - 1));
		if (intr_count > 3)
			return intr_count;

		buf_id += (intr_count - 1);
		/* Reprogram registers*/
		for (i = buff_done; i < buf_id; i++) {
			int j = i % 4;
			buf_size = intelhaddata->buf_info[j].buf_size;
			buf_addr = intelhaddata->buf_info[j].buf_addr;
			had_write_register(AUD_BUF_A_LENGTH +
					(j * HAD_REG_WIDTH), buf_size);
			had_write_register(
					AUD_BUF_A_ADDR+(j * HAD_REG_WIDTH),
					(buf_addr | BIT(0) | BIT(1)));
		}
		buf_id = buf_id % 4;
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
		intelhaddata->buff_done = buf_id;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
	}

	return intr_count;
}

static inline int had_start_dummy_playback(struct snd_intelhad *intelhaddata)
{
	int retval = 0;
	enum intel_had_aud_buf_type buf_id;
	u32 buf_size;
	struct snd_pcm_substream *substream;
	struct had_pvt_data *had_stream;
	u32 msecs, rate, channels;
	unsigned long flag_irqs;

	pr_debug("Enter:%s", __func__);

	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	had_stream = intelhaddata->private_data;
	buf_id = intelhaddata->curr_buf;
	substream = intelhaddata->stream_info.had_substream;

	if ((had_stream->stream_status < HAD_RUNNING_DUMMY) && substream
			&& substream->runtime) {
		had_stream->stream_status = HAD_RUNNING_DUMMY;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_debug("Steam active, start dummy_playback\n");
		/* In case substream is active, start reporting
		 * dummy buffer_done interrupts to above ALSA layer.
		 */
		buf_size =  intelhaddata->buf_info[buf_id].buf_size;
		rate = substream->runtime->rate;
		channels = substream->runtime->channels;
		msecs = (buf_size*1000)/(rate*channels*4);
		intelhaddata->timer = msecs_to_jiffies(msecs);
		schedule_delayed_work(
				&intelhaddata->dummy_audio,
				intelhaddata->timer);
	} else
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);

	return retval;
}

static inline int had_process_pre_start(struct snd_intelhad *intelhaddata,
		enum intel_had_aud_buf_type buf_id)
{
	int retval = 0;
	struct had_pvt_data *had_stream;
	enum intel_had_aud_buf_type prg_start, prg_end;
	enum intel_had_aud_buf_type inv_start;
	unsigned long flag_irqs;

	had_stream = intelhaddata->private_data;

	if (intelhaddata->valid_buf_cnt-1 == buf_id) {
		if (had_stream->stream_status >= HAD_RUNNING_STREAM)
			buf_id = HAD_BUF_TYPE_A;
		else	/* Use only silence buffers C & D */
			buf_id = HAD_BUF_TYPE_C;
	} else
		buf_id = buf_id + 1;

	/* Program all buffers in case of Buf == A */
	if (buf_id == HAD_BUF_TYPE_A) {
		prg_start = buf_id + 1;
		prg_end = HAD_BUF_TYPE_D;
	} else {
		prg_start = HAD_BUF_TYPE_A;
		prg_end = buf_id - 1;
	}
	retval = snd_intelhad_prog_buffer(intelhaddata,
			prg_start, prg_end);
	/* Invalidate all buffs after buf_id */
	inv_start = buf_id + 1;
	if ((buf_id != HAD_BUF_TYPE_A) && (buf_id != HAD_BUF_TYPE_D))
		snd_intelhad_invd_buffer(inv_start, HAD_BUF_TYPE_D);

	/* Start reporting BUFFER_DONE/UNDERRUN to above layers*/
	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	intelhaddata->curr_buf = buf_id;

	if (buf_id != HAD_BUF_TYPE_A)
		had_stream->process_trigger = START_TRIGGER;
	else {
		had_stream->stream_status = HAD_RUNNING_STREAM;
		had_stream->process_trigger = NO_TRIGGER;
	}
	spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);

	pr_debug("Processed %s, buf_id = %d\n", __func__, buf_id);

	return retval;
}

static inline int had_process_start_trigger(struct snd_intelhad *intelhaddata)
{
	int retval = 0;
	struct had_pvt_data *had_stream;
	enum intel_had_aud_buf_type prg_start;
	unsigned long flag_irqs;

	had_stream = intelhaddata->private_data;
	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	/* Check for device state
	 * In case, _DISCONNECTED, start dummy_playback
	 * if not done
	 */
	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_debug("Cable plugged-out\n");
		retval = had_start_dummy_playback(intelhaddata);
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
		had_stream->process_trigger = NO_TRIGGER;
		intelhaddata->curr_buf = HAD_BUF_TYPE_A;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		return retval;
	}

	had_stream->stream_status = HAD_RUNNING_STREAM;
	had_stream->process_trigger = NO_TRIGGER;
	intelhaddata->curr_buf = HAD_BUF_TYPE_A;
	prg_start = intelhaddata->buff_done;
	spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
	retval = snd_intelhad_prog_buffer(intelhaddata,
			prg_start, HAD_BUF_TYPE_D);

	return retval;
}

static inline int had_process_stop_trigger(struct snd_intelhad *intelhaddata)
{
	int i, retval = 0;
	struct had_pvt_data *had_stream;
	enum intel_had_aud_buf_type buf_id;
	u32 buf_addr;

	had_stream = intelhaddata->private_data;

	buf_id = intelhaddata->curr_buf;
	/* If device disconnected, ignore this interrupt */
	if (had_stream->stream_status == HAD_RUNNING_DUMMY) {
		had_stream->stream_status = HAD_INIT;
		had_stream->process_trigger = NO_TRIGGER;
		return retval;
	}

	/* All successive intr for Silence stream */
	had_stream->stream_status = HAD_RUNNING_SILENCE;
	had_stream->process_trigger = NO_TRIGGER;

	/* If buf_id < HAD_BUF_TYPE_C, ignore */
	if (buf_id < HAD_BUF_TYPE_C) {
		intelhaddata->curr_buf = HAD_BUF_TYPE_C;
		return retval;
	}
	if (buf_id == HAD_BUF_TYPE_C)
		intelhaddata->curr_buf = HAD_BUF_TYPE_D;
	else
		intelhaddata->curr_buf = HAD_BUF_TYPE_C;

	/* _STOP -> _START -> _STOP occured
	 * invalidate Buff_A, Buff_B
	 */
	snd_intelhad_invd_buffer(HAD_BUF_TYPE_A, HAD_BUF_TYPE_B);
	i = buf_id;
	buf_addr = virt_to_phys(intelhaddata->flat_data);
	/* Program the buf registers with addr and len */
	had_write_register(AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
			buf_addr | BIT(0) | BIT(1));
	had_write_register(AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
			(MAX_SZ_ZERO_BUF));

	intelhaddata->buf_info[i].buf_addr = buf_addr;
	intelhaddata->buf_info[i].buf_size = MAX_SZ_ZERO_BUF;
	intelhaddata->buf_info[i].is_valid = true;
	pr_debug("buf[%d] addr=%#x  and size=%d\n", i,
			intelhaddata->buf_info[i].buf_addr,
			intelhaddata->buf_info[i].buf_size);

	return retval;
}

int had_process_buffer_done(struct snd_intelhad *intelhaddata)
{
	int retval = 0;
	u32 len = 1;
	enum intel_had_aud_buf_type buf_id;
	enum intel_had_aud_buf_type buff_done;
	struct pcm_stream_info *stream;
	u32 buf_size;
	struct had_pvt_data *had_stream;
	int intr_count;
	enum had_stream_status		stream_status;
	enum had_process_trigger	process_trigger;
	unsigned long flag_irqs;

	had_stream = intelhaddata->private_data;
	stream = &intelhaddata->stream_info;
	intr_count = 1;

	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	buf_id = intelhaddata->curr_buf;
	intelhaddata->buff_done = buf_id;
	buff_done = intelhaddata->buff_done;
	buf_size = intelhaddata->buf_info[buf_id].buf_size;
	stream_status = had_stream->stream_status;
	process_trigger = had_stream->process_trigger;

	pr_debug("Enter:%s buf_id=%d", __func__, buf_id);

	/* Every debug statement has an implication
	 * of ~5msec. Thus, avoid having >3 debug statements
	 * for each buffer_done handling.
	 */

	/* Check for any intr_miss in case of active playback */
	if ((had_stream->stream_status == HAD_RUNNING_STREAM) &&
			(had_stream->process_trigger == NO_TRIGGER) &&
			!flag_en_allbufs) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		intr_count = had_chk_intrmiss(intelhaddata, buf_id);
		if (!intr_count || (intr_count > 3)) {
			pr_err("HAD SW state in non-recoverable!!! mode\n");
			pr_err("Already played stale data\n");
			return retval;
		}
		buf_id += (intr_count - 1);
		buf_id = buf_id % 4;
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	}

	/* Acknowledge _START trigger recieved */
	if (had_stream->process_trigger == PRE_START) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		retval = had_process_pre_start(intelhaddata, buf_id);
		return retval;
	}

	/* Program actual data buffer, in case _START trigger recieved */
	if (had_stream->process_trigger == START_TRIGGER) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		retval = had_process_start_trigger(intelhaddata);
		return retval;
	}

	/* Program Silence buffer, in case _STOP trigger recieved */
	if (had_stream->process_trigger == STOP_TRIGGER) {
		/* Process with spin_lock acquired
		 * Can be optimized later
		 */
		retval = had_process_stop_trigger(intelhaddata);
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		pr_debug("Processed stop trigger in hanlder:bufid=%d\n",
				buf_id);
		return retval;
	}

	intelhaddata->buf_info[buf_id].is_valid = true;
	if (intelhaddata->valid_buf_cnt-1 == buf_id) {
		if (had_stream->stream_status >= HAD_RUNNING_STREAM)
			intelhaddata->curr_buf = HAD_BUF_TYPE_A;
		else	/* Use only silence buffers C & D */
			intelhaddata->curr_buf = HAD_BUF_TYPE_C;
	} else
		intelhaddata->curr_buf = buf_id + 1;

	if (had_stream->stream_status == HAD_RUNNING_DUMMY) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		goto exit;
	}

	spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);

	if (had_get_hwstate(intelhaddata)) {
		pr_err("HDMI cable plugged-out\n");
		return retval;
	}

	if (flag_en_allbufs) {
		pr_debug("Enabling Top half buffers after _PLUG\n");
		retval = snd_intelhad_prog_buffer(intelhaddata,
				HAD_BUF_TYPE_A, (buf_id-1));
		flag_en_allbufs = 0;
	}
	/*Reprogram the registers with addr and length*/
	had_write_register(AUD_BUF_A_LENGTH +
			(buf_id * HAD_REG_WIDTH), buf_size);
	had_write_register(AUD_BUF_A_ADDR+(buf_id * HAD_REG_WIDTH),
			intelhaddata->buf_info[buf_id].buf_addr|
			BIT(0) | BIT(1));

	had_read_register(AUD_BUF_A_LENGTH + (buf_id * HAD_REG_WIDTH),
					&len);
	pr_debug("%s:Enabled buf[%d]\n", __func__, buf_id);
exit:
	/* In case of actual/dummy data,
	 * report buffer_done to above ALSA layer
	 */
	buf_size =  intelhaddata->buf_info[buf_id].buf_size;
	if (stream_status >= HAD_RUNNING_STREAM) {
		intelhaddata->stream_info.buffer_rendered +=
			(intr_count * buf_size);
		stream->period_elapsed(stream->had_substream);
	}

	return retval;
}

int had_process_buffer_underrun(struct snd_intelhad *intelhaddata)
{
	int i = 0, retval = 0;
	enum intel_had_aud_buf_type buf_id;
	struct pcm_stream_info *stream;
	struct had_pvt_data *had_stream;
	enum had_stream_status		stream_status;
	enum had_process_trigger	process_trigger;
	u32 hdmi_status;
	unsigned long flag_irqs;

	had_stream = intelhaddata->private_data;
	stream = &intelhaddata->stream_info;

	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	buf_id = intelhaddata->curr_buf;
	stream_status = had_stream->stream_status;
	process_trigger = had_stream->process_trigger;
	intelhaddata->buff_done = buf_id;
	if (stream_status == HAD_RUNNING_STREAM)
		intelhaddata->curr_buf = HAD_BUF_TYPE_A;
	else	/* Use only silence buffers C & D */
		intelhaddata->curr_buf = HAD_BUF_TYPE_C;

	spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);

	pr_debug("Enter:%s buf_id=%d, stream_status=%d, process_trigger=%d\n",
			__func__, buf_id, stream_status, process_trigger);

	if (stream_status == HAD_RUNNING_DUMMY) {
		pr_err("_UNDERRUN occured during dummy playback\n");
		return retval;
	}

	/* Handle Underrun interrupt within Audio Unit */
	had_write_register(AUD_CONFIG, 0);
	/* Reset buffer pointers */
	had_write_register(AUD_HDMI_STATUS, 1);
	had_write_register(AUD_HDMI_STATUS, 0);
	/**
	 * The interrupt status 'sticky' bits might not be cleared by
	 * setting '1' to that bit once...
	 */
	do { /* clear bit30, 31 AUD_HDMI_STATUS */
		had_read_register(AUD_HDMI_STATUS, &hdmi_status);
		pr_debug("HDMI status =0x%x\n", hdmi_status);
		if (hdmi_status & AUD_CONFIG_MASK_UNDERRUN) {
			i++;
			hdmi_status &= (AUD_CONFIG_MASK_SRDBG |
					AUD_CONFIG_MASK_FUNCRST);
			hdmi_status |= ~AUD_CONFIG_MASK_UNDERRUN;
			had_write_register(AUD_HDMI_STATUS, hdmi_status);
		} else
			break;
	} while (i < MAX_CNT);
	if (i >= MAX_CNT)
		pr_err("Unable to clear UNDERRUN bits\n");

	if (stream_status == HAD_RUNNING_STREAM) {
		if (process_trigger == NO_TRIGGER) {
			/* Report UNDERRUN error to above layers */
			intelhaddata->flag_underrun = 1;
			stream->period_elapsed(stream->had_substream);
			had_read_modify(AUD_CONFIG, 1, BIT(0));
			retval = snd_intelhad_prog_buffer(intelhaddata,
					HAD_BUF_TYPE_A, HAD_BUF_TYPE_D);
		} else if (process_trigger == STOP_TRIGGER) {
			spin_lock_irqsave(&intelhaddata->had_spinlock,
					flag_irqs);
			had_stream->process_trigger = NO_TRIGGER;
			spin_unlock_irqrestore(&intelhaddata->had_spinlock,
					flag_irqs);
			/* Invalidate A & B */
			snd_intelhad_invd_buffer(HAD_BUF_TYPE_A,
					HAD_BUF_TYPE_B);
			/* Start sending silence data */
			retval = snd_intelhad_start_silence(intelhaddata);
			return retval;
		} else {
			pr_err("%s:Should never come here\n", __func__);
			pr_err("stream_status=%d,process_trigger=%d\n",
					stream_status, process_trigger);
			return retval;
		}
	} else if (stream_status == HAD_RUNNING_SILENCE) {
		if (process_trigger < START_TRIGGER)
			retval = snd_intelhad_start_silence(intelhaddata);
		else if (process_trigger == START_TRIGGER) {
			spin_lock_irqsave(&intelhaddata->had_spinlock,
					flag_irqs);
			intelhaddata->curr_buf = HAD_BUF_TYPE_A;
			had_stream->process_trigger = NO_TRIGGER;
			had_stream->stream_status = HAD_RUNNING_STREAM;
			spin_unlock_irqrestore(&intelhaddata->had_spinlock,
					flag_irqs);
			had_read_modify(AUD_CONFIG, 1, BIT(0));
			retval = snd_intelhad_prog_buffer(intelhaddata,
				HAD_BUF_TYPE_A, HAD_BUF_TYPE_D);
			/* Pre start already processed */
		} else {
			pr_err("%s:Should never come here\n", __func__);
			pr_err("stream_status=%d,process_trigger=%d\n",
					stream_status, process_trigger);
			return retval;
		}
	}

	return retval;
}

int had_process_hot_plug(struct snd_intelhad *intelhaddata)
{
	int caps, retval = 0;
	enum intel_had_aud_buf_type buf_id;
	struct snd_pcm_substream *substream;
	struct had_pvt_data *had_stream;
	enum had_stream_status		stream_status;
	unsigned long flag_irqs;

	pr_debug("Enter:%s", __func__);

	substream = intelhaddata->stream_info.had_substream;
	had_stream = intelhaddata->private_data;

	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	if (intelhaddata->drv_status == HAD_DRV_CONNECTED) {
		pr_debug("Device already connected\n");
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		return retval;
	}
	buf_id = intelhaddata->curr_buf;
	intelhaddata->buff_done = buf_id;
	intelhaddata->drv_status = HAD_DRV_CONNECTED;
	buf_id = intelhaddata->curr_buf;
	stream_status = had_stream->stream_status;
	spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);

	pr_debug("Processing HOT_PLUG, buf_id = %d\n", buf_id);
	if (stream_status == HAD_INIT) {
		/* Start sending silence data */
		pr_debug("Start sending SILENCE\n");
		caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		retval = had_set_caps(HAD_SET_ENABLE_AUDIO_INT, &caps);
		retval = had_set_caps(HAD_SET_ENABLE_AUDIO, NULL);
		retval = snd_intelhad_configure_silence(intelhaddata);
		retval = snd_intelhad_start_silence(intelhaddata);
		return retval;
	}

	/* Safety check */
	if (!substream) {
		pr_err("PANIC!!! Should never come here\n");
		return retval;
	}

	if (stream_status == HAD_RUNNING_DUMMY) {
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
		had_stream->stream_status = HAD_RUNNING_STREAM;
		flag_en_allbufs = 1;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		cancel_delayed_work(&intelhaddata->dummy_audio);
	}
	caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
	retval = had_set_caps(HAD_SET_ENABLE_AUDIO_INT, &caps);
	retval = had_set_caps(HAD_SET_ENABLE_AUDIO, NULL);

	/* Reset HW within Audio unit */
	had_write_register(AUD_CONFIG, 0);
	had_write_register(AUD_HDMI_STATUS, 1);
	had_write_register(AUD_HDMI_STATUS, 0);

	/*Reprogram the registers with addr and length*/
	pr_debug("Enabling Bottom half buffers after _PLUG\n");
	/* If all buffs already enabled,
	 * No need to enable Top half buffers
	 */
	if (buf_id == HAD_BUF_TYPE_A)
		flag_en_allbufs = 0;
	retval = snd_intelhad_prog_buffer(intelhaddata, buf_id, HAD_BUF_TYPE_D);

	if (substream) { /* continue transfer */
		snd_intelhad_init_audio_ctrl(substream,
				intelhaddata, 0);
		hdmi_audio_mode_change(substream);
	}
	if (buf_id == HAD_BUF_TYPE_D) { /* Enable all remaining buffs now*/
		retval = snd_intelhad_prog_buffer(intelhaddata, HAD_BUF_TYPE_A,
				(buf_id-1));
		flag_en_allbufs = 0;
	}

	return retval;
}

int had_process_hot_unplug(struct snd_intelhad *intelhaddata)
{
	int caps, retval = 0;
	enum intel_had_aud_buf_type buf_id;
	struct snd_pcm_substream *substream;
	struct had_pvt_data *had_stream;
	unsigned long flag_irqs;

	pr_debug("Enter:%s", __func__);

	had_stream = intelhaddata->private_data;
	buf_id = intelhaddata->curr_buf;
	substream = intelhaddata->stream_info.had_substream;

	spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED) {
		pr_debug("Device already disconnected\n");
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		return retval;
	} else {
		/* Disable Audio */
		caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		retval = had_set_caps(HAD_SET_DISABLE_AUDIO_INT, &caps);
		retval = had_set_caps(HAD_SET_DISABLE_AUDIO, NULL);
		had_read_modify(AUD_CONFIG, 0, BIT(0));
	}

	intelhaddata->drv_status = HAD_DRV_DISCONNECTED;

	if (had_stream->stream_status == HAD_RUNNING_STREAM) {
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		had_start_dummy_playback(intelhaddata);
	} else {
		had_stream->stream_status = HAD_INIT;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
	}

	return retval;
}

/**
 * had_event_handler - Call back function to handle events
 *
 * @event_type: Event type to handle
 * @data: data related to the event_type
 *
 * This function is invoked to handle HDMI events from client driver.
 */
int had_event_handler(enum had_event_type event_type, void *data)
{
	int retval = 0;
	struct snd_intelhad *intelhaddata = data;
	enum intel_had_aud_buf_type buf_id;
	struct snd_pcm_substream *substream;
	struct had_pvt_data *had_stream;
	unsigned long flag_irqs;

	buf_id = intelhaddata->curr_buf;
	had_stream = intelhaddata->private_data;

	/* Switching to a function can drop atomicity even in INTR context.
	 * Thus, a big lock is acquired to maintain atomicity.
	 * This can be optimized later.
	 * Currently, only buffer_done/_underrun executes in INTR context.
	 * Also, locking is implemented seperately to avoid real contention
	 * of data(struct intelhaddata) between IRQ/SOFT_IRQ/PROCESS context.
	 */
	substream = intelhaddata->stream_info.had_substream;
	switch (event_type) {
	case HAD_EVENT_AUDIO_BUFFER_DONE:
		retval = had_process_buffer_done(intelhaddata);
	break;

	case HAD_EVENT_AUDIO_BUFFER_UNDERRUN:
		retval = had_process_buffer_underrun(intelhaddata);
	break;

	case HAD_EVENT_HOT_PLUG:
		retval = had_process_hot_plug(intelhaddata);
	break;

	case HAD_EVENT_HOT_UNPLUG:
		retval = had_process_hot_unplug(intelhaddata);
	break;

	case HAD_EVENT_MODE_CHANGING:
		pr_debug(" called _event_handler with _MODE_CHANGE event\n");
		/* Process only if stream is active & cable Plugged-in */
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irqs);
		if (intelhaddata->drv_status >= HAD_DRV_DISCONNECTED) {
			spin_unlock_irqrestore(&intelhaddata->had_spinlock,
					flag_irqs);
			break;
		}
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irqs);
		if ((had_stream->stream_status == HAD_RUNNING_STREAM)
				&& substream)
			retval = hdmi_audio_mode_change(substream);
	break;

	default:
		pr_debug("error un-handled event !!\n");
		retval = -EINVAL;
	break;

	}
	return retval;
}
