/*
 *   intel_mid_hdmi_audio.c - Intel HDMI audio driver for MID
 *
 *  Copyright (C) 2010 Intel Corp
 *  Authors:	Sailaja Bandarupalli <sailaja.bandarupalli@intel.com>
 *		Ramesh Babu K V	<ramesh.babu@intel.com>
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
 * ALSA driver for Intel MID HDMI audio controller
 */

#define pr_fmt(fmt)	"had: " fmt

#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/initval.h>
#include "intel_mid_hdmi_audio.h"

#define PCM_INDEX		0
#define MAX_PB_STREAMS		1
#define MAX_CAP_STREAMS		0
#define HDMI_AUDIO_DRIVER	"hdmi-audio"
static DEFINE_MUTEX(had_mutex);

/*standard module options for ALSA. This module supports only one card*/
static int hdmi_card_index = SNDRV_DEFAULT_IDX1;
static char *hdmi_card_id = SNDRV_DEFAULT_STR1;
static struct snd_intelhad *had_pvt_data;

module_param(hdmi_card_index, int, 0444);
MODULE_PARM_DESC(hdmi_card_index,
		"Index value for INTEL Intel HDMI Audio controller.");
module_param(hdmi_card_id, charp, 0444);
MODULE_PARM_DESC(hdmi_card_id,
		"ID string for INTEL Intel HDMI Audio controller.");
MODULE_AUTHOR("Sailaja Bandarupalli <sailaja.bandarupalli@intel.com>");
MODULE_AUTHOR("Ramesh Babu K V <ramesh.babu@intel.com>");
MODULE_AUTHOR("Vaibhav Agarwal <vaibhav.agarwal@intel.com>");
MODULE_DESCRIPTION("Intel HDMI Audio driver");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("{Intel,Intel_HAD}");
MODULE_VERSION(HAD_DRIVER_VERSION);

#define INFO_FRAME_WORD1	0x000a0184
#define FIFO_THRESHOLD		0xFE
#define BYTES_PER_WORD		0x4
#define CH_STATUS_MAP_32KHZ	0x3
#define CH_STATUS_MAP_44KHZ	0x0
#define CH_STATUS_MAP_48KHZ	0x2
#define MAX_SMPL_WIDTH_20	0x0
#define MAX_SMPL_WIDTH_24	0x1
#define SMPL_WIDTH_16BITS	0x1
#define SMPL_WIDTH_24BITS	0x5
#define CHANNEL_ALLOCATION	0x1F
#define MASK_BYTE0		0x000000FF
#define VALID_DIP_WORDS		3
#define LAYOUT0			0
#define LAYOUT1			1

/* hardware capability structure */
static const struct snd_pcm_hardware snd_intel_hadstream = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_DOUBLE |
		SNDRV_PCM_INFO_MMAP|
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH),
	.formats = (SNDRV_PCM_FMTBIT_S24 |
		SNDRV_PCM_FMTBIT_U24),
	.rates = SNDRV_PCM_RATE_32000 |
		SNDRV_PCM_RATE_44100 |
		SNDRV_PCM_RATE_48000 |
		SNDRV_PCM_RATE_88200 |
		SNDRV_PCM_RATE_96000 |
		SNDRV_PCM_RATE_176400 |
		SNDRV_PCM_RATE_192000,
	.rate_min = HAD_MIN_RATE,
	.rate_max = HAD_MAX_RATE,
	.channels_min = HAD_MIN_CHANNEL,
	.channels_max = HAD_MAX_CHANNEL,
	.buffer_bytes_max = HAD_MAX_PERIOD_BYTES,
	.period_bytes_min = HAD_MIN_PERIOD_BYTES,
	.period_bytes_max = HAD_MAX_BUFFER,
	.periods_min = HAD_MIN_PERIODS,
	.periods_max = HAD_MAX_PERIODS,
	.fifo_size = HAD_FIFO_SIZE,
};

/**
 * snd_intelhad_init_audio_ctrl - to initialize audio channel status
 * registers and confgiuration registers
 *
 * @substream:substream for which the prepare function is called
 * @intelhaddata:substream private data
 *
 * This function is called in the prepare callback
 */
int snd_intelhad_init_audio_ctrl(struct snd_pcm_substream *substream,
					struct snd_intelhad *intelhaddata,
					int flag_silence)
{
	union aud_cfg cfg_val = {.cfg_regval = 0};
	union aud_ch_status_0 ch_stat0 = {.status_0_regval = 0};
	union aud_ch_status_1 ch_stat1 = {.status_1_regval = 0};
	union aud_buf_config buf_cfg = {.buf_cfgval = 0};
	u8 channels;
	int rate, format;

	ch_stat0.status_0_regx.lpcm_id = (intelhaddata->aes_bits &
						IEC958_AES0_NONAUDIO)>>1;
	ch_stat0.status_0_regx.clk_acc = (intelhaddata->aes_bits &
						IEC958_AES3_CON_CLOCK)>>4;
	if (flag_silence)
		rate = AUD_SAMPLE_RATE_44_1;
	else
		rate = substream->runtime->rate;

	switch (rate) {
	case AUD_SAMPLE_RATE_32:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_32KHZ;
	break;

	case AUD_SAMPLE_RATE_44_1:
	case AUD_SAMPLE_RATE_88_2:
	case AUD_SAMPLE_RATE_176_4:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_44KHZ;
	break;

	case AUD_SAMPLE_RATE_48:
	case AUD_SAMPLE_RATE_96:
	case HAD_MAX_RATE:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_48KHZ;
	break;

	default:
		return -EINVAL;
	break;

	}
	intelhaddata->reg_ops.hdmi_audio_write_register(
				AUD_CH_STATUS_0, ch_stat0.status_0_regval);

	if (flag_silence)
		format = SNDRV_PCM_FORMAT_S24_LE;
	else
		format = substream->runtime->format;

	if (format == SNDRV_PCM_FORMAT_S16_LE) {
		ch_stat1.status_1_regx.max_wrd_len = MAX_SMPL_WIDTH_20;
		ch_stat1.status_1_regx.wrd_len = SMPL_WIDTH_16BITS;
	} else if (format == SNDRV_PCM_FORMAT_S24_LE) {
		ch_stat1.status_1_regx.max_wrd_len = MAX_SMPL_WIDTH_24;
		ch_stat1.status_1_regx.wrd_len = SMPL_WIDTH_24BITS;
	} else {
		ch_stat1.status_1_regx.max_wrd_len = 0;
		ch_stat1.status_1_regx.wrd_len = 0;
	}
	intelhaddata->reg_ops.hdmi_audio_write_register(
				AUD_CH_STATUS_1, ch_stat1.status_1_regval);

	buf_cfg.buf_cfg_regx.fifo_width = FIFO_THRESHOLD;
	buf_cfg.buf_cfg_regx.aud_delay = 0;
	intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_CONFIG, buf_cfg.buf_cfgval);

	if (flag_silence)
		channels = HAD_MIN_CHANNEL;
	else
		channels = substream->runtime->channels;


	switch (channels) {
	case 1:
	case 2:
		cfg_val.cfg_regx.num_ch = CH_STEREO;
		cfg_val.cfg_regx.layout = LAYOUT0;
	break;

	case 3:
	case 4:
		cfg_val.cfg_regx.num_ch = CH_THREE_FOUR;
		cfg_val.cfg_regx.layout = LAYOUT1;
	break;

	case 5:
	case 6:
		cfg_val.cfg_regx.num_ch = CH_FIVE_SIX;
		cfg_val.cfg_regx.layout = LAYOUT1;
	break;

	case 7:
	case 8:
		cfg_val.cfg_regx.num_ch = CH_SEVEN_EIGHT;
		cfg_val.cfg_regx.layout = LAYOUT1;
	break;

	}

	cfg_val.cfg_regx.val_bit = 1;
	intelhaddata->reg_ops.hdmi_audio_write_register(
						AUD_CONFIG, cfg_val.cfg_regval);
	return 0;
}

/**
 * snd_intelhad_prog_dip - to initialize Data Island Packets registers
 *
 * @substream:substream for which the prepare function is called
 * @intelhaddata:substream private data
 *
 * This function is called in the prepare callback
 */
static void snd_intelhad_prog_dip(struct snd_pcm_substream *substream,
				struct snd_intelhad *intelhaddata,
				int flag_silence)
{
	int i;
	union aud_ctrl_st ctrl_state = {.ctrl_val = 0};
	union aud_info_frame2 frame2 = {.fr2_val = 0};
	union aud_info_frame3 frame3 = {.fr3_val = 0};
	u8 checksum = 0;

	intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_CNTL_ST, ctrl_state.ctrl_val);

	if (flag_silence)
		frame2.fr2_regx.chnl_cnt = HAD_MIN_CHANNEL - 1;
	else
		frame2.fr2_regx.chnl_cnt = substream->runtime->channels - 1;

	/* Set to stereo */
	frame3.fr3_regx.chnl_alloc = 0;

	/*Calculte the byte wide checksum for all valid DIP words*/
	for (i = 0; i < BYTES_PER_WORD; i++)
		checksum += (INFO_FRAME_WORD1 >> i*BITS_PER_BYTE) & MASK_BYTE0;
	for (i = 0; i < BYTES_PER_WORD; i++)
		checksum += (frame2.fr2_val >> i*BITS_PER_BYTE) & MASK_BYTE0;
	for (i = 0; i < BYTES_PER_WORD; i++)
		checksum += (frame3.fr3_val >> i*BITS_PER_BYTE) & MASK_BYTE0;

	frame2.fr2_regx.chksum = -(checksum);

	intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_HDMIW_INFOFR, INFO_FRAME_WORD1);
	intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_HDMIW_INFOFR, frame2.fr2_val);
	intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_HDMIW_INFOFR, frame3.fr3_val);
	/* program remaining DIP words with zero */
	for (i = 0; i < HAD_MAX_DIP_WORDS-VALID_DIP_WORDS; i++)
		intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_HDMIW_INFOFR, 0x0);

	ctrl_state.ctrl_regx.dip_freq = 1;
	ctrl_state.ctrl_regx.dip_en_sta = 1;
	intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_CNTL_ST, ctrl_state.ctrl_val);
}

/**
 * snd_intelhad_prog_buffer - programs buffer
 * address and length registers
 *
 * @substream:substream for which the prepare function is called
 * @intelhaddata:substream private data
 *
 * This function programs ring buffer address and length into registers.
 */
int snd_intelhad_prog_buffer(struct snd_intelhad *intelhaddata,
					int start, int end)
{
	u32 ring_buf_addr, ring_buf_size, period_bytes;
	u8 i, num_periods;
	struct snd_pcm_substream *substream;

	substream = intelhaddata->stream_info.had_substream;
	ring_buf_addr = virt_to_phys(substream->runtime->dma_area);
	ring_buf_size = snd_pcm_lib_buffer_bytes(substream);
	intelhaddata->stream_info.ring_buf_size = ring_buf_size;
	period_bytes = frames_to_bytes(substream->runtime,
				substream->runtime->period_size);
	num_periods = substream->runtime->periods;

	pr_debug("Ring buffer address = %#x\n", ring_buf_addr);
	pr_debug("Ring buffer size = %#x\n", ring_buf_size);
	pr_debug("period size in bytes = %d\n", period_bytes);

	/* buffer addr should  be 64 byte aligned, period bytes
	 will be used to calculate addr offset*/
	period_bytes &= ~0x3F;
	pr_debug("period size in bytes after align = %d\n", period_bytes);

	/* Hardware supports MAX_PERIODS buffers */
	if (end >= HAD_MAX_PERIODS)
		return -EINVAL;

	for (i = start; i <= end; i++) {
		/* Program the buf registers with addr and len */
		intelhaddata->buf_info[i].buf_addr = ring_buf_addr +
							 (i * period_bytes);
		if (i < num_periods-1)
			intelhaddata->buf_info[i].buf_size = period_bytes;
		else
			intelhaddata->buf_info[i].buf_size = ring_buf_size -
							(period_bytes*i);

		intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
					intelhaddata->buf_info[i].buf_addr |
					BIT(0) | BIT(1));
		intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					period_bytes);
		intelhaddata->buf_info[i].is_valid = true;
		pr_debug("buf[%d] addr=%#x  and size=%d\n", i,
				intelhaddata->buf_info[i].buf_addr,
				intelhaddata->buf_info[i].buf_size);
	}
	intelhaddata->valid_buf_cnt = num_periods;
	return 0;
}

inline void snd_intelhad_read_len(struct snd_intelhad *intelhaddata)
{
	int i;
	u32 len;
	for (i = 0; i < 4 ; i++) {
		/* Program the buf registers with addr and len */
		intelhaddata->reg_ops.hdmi_audio_read_register(
					AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					&len);
		pr_debug("buf[%d] size=%d\n", i, len);
	}
}

/**
 * snd_intelhad_prog_cts - Program HDMI audio CTS value
 *
 * @aud_samp_freq: sampling frequency of audio data
 * @tmds: sampling frequency of the display data
 * @n_param: N value, depends on aud_samp_freq
 * @intelhaddata:substream private data
 *
 * Program CTS register based on the audio and display sampling frequency
 */
static void snd_intelhad_prog_cts(u32 aud_samp_freq, u32 tmds, u32 n_param,
				struct snd_intelhad *intelhaddata)
{
	u32 cts_val;
	u64 dividend, divisor;

	/* Calculate CTS according to HDMI 1.3a spec*/
	dividend = (u64)tmds * n_param*1000;
	divisor = 128 * aud_samp_freq;
	cts_val = div64_u64(dividend, divisor);
	pr_debug("CTS Value=%d\n", cts_val);
	intelhaddata->reg_ops.hdmi_audio_write_register(
				AUD_HDMI_CTS, (BIT(20) | cts_val));
}

/**
 * snd_intelhad_prog_n - Program HDMI audio N value
 *
 * @aud_samp_freq: sampling frequency of audio data
 * @n_param: N value, depends on aud_samp_freq
 * @intelhaddata:substream private data
 *
 * This function is called in the prepare callback.
 * It programs based on the audio and display sampling frequency
 */
static int snd_intelhad_prog_n(u32 aud_samp_freq, u32 *n_param,
				struct snd_intelhad *intelhaddata)
{
	u32 n_val;
	int retval = 0;

	/* Select N according to HDMI 1.3a spec*/
	switch (aud_samp_freq) {
	case AUD_SAMPLE_RATE_32:
		n_val = 4096;
	break;

	case AUD_SAMPLE_RATE_44_1:
		n_val = 6272;
	break;

	case AUD_SAMPLE_RATE_48:
		n_val = 6144;
	break;

	case AUD_SAMPLE_RATE_88_2:
		n_val = 12544;
	break;

	case AUD_SAMPLE_RATE_96:
		n_val = 12288;
	break;

	case AUD_SAMPLE_RATE_176_4:
		n_val = 25088;
	break;

	case HAD_MAX_RATE:
		n_val = 24576;
	break;

	default:
		retval = -EINVAL;
	break;

	}
	if (retval)
		return retval;
	intelhaddata->reg_ops.hdmi_audio_write_register(
				AUD_N_ENABLE, (BIT(20) | n_val));
	*n_param = n_val;
	return retval;
}

/**
* snd_intelhad_open - stream initializations are done here
* @substream:substream for which the stream function is called
*
* This function is called whenever a PCM stream is opened
*/
static int snd_intelhad_open(struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata;
	struct snd_pcm_runtime *runtime;
	struct had_stream_pvt *stream;
	int retval;

	pr_debug("snd_intelhad_open called\n");
	intelhaddata = snd_pcm_substream_chip(substream);

	mutex_lock(&intelhaddata->had_lock);
	if (intelhaddata->drv_status != HAD_DRV_CONNECTED) {
		pr_err("had in disconnected/suspended state :%d\n",
				intelhaddata->drv_status);
		mutex_unlock(&intelhaddata->had_lock);
		return -ENODEV;
	}

	if (intelhaddata->playback_cnt > 0) {
		mutex_unlock(&intelhaddata->had_lock);
		return -EBUSY;
	} else
		intelhaddata->playback_cnt++;

	mutex_unlock(&intelhaddata->had_lock);

	runtime = substream->runtime;
	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	runtime->hw = snd_intel_hadstream;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream) {
		retval = -ENOMEM;
		goto exit_err;
	}
	stream->stream_status = STREAM_INIT;
	runtime->private_data = stream;
	retval = snd_pcm_hw_constraint_integer(runtime,
			 SNDRV_PCM_HW_PARAM_PERIODS);
	if (retval < 0) {
		kfree(stream);
		goto exit_err;
	}
	mutex_lock(&intelhaddata->had_lock);
	intelhaddata->drv_status = HAD_DRV_RUNNING;
	mutex_unlock(&intelhaddata->had_lock);
	return retval;
exit_err:
	mutex_lock(&intelhaddata->had_lock);
	intelhaddata->playback_cnt--;
	mutex_unlock(&intelhaddata->had_lock);
	return retval;
}

/**
* had_period_elapsed - updates the hardware pointer status
* @had_substream:substream for which the stream function is called
*
*/
static void had_period_elapsed(void *had_substream)
{
	struct snd_pcm_substream *substream = had_substream;
	struct had_stream_pvt *stream;

	if (!substream || !substream->runtime)
		return;
	stream = substream->runtime->private_data;
	if (!stream)
		return;

	if (stream->stream_status != STREAM_RUNNING)
		return;
	snd_pcm_period_elapsed(substream);
	return;
}

/**
* snd_intelhad_init_stream - internal function to initialize stream info
* @substream:substream for which the stream function is called
*
*/
static int snd_intelhad_init_stream(struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata = snd_pcm_substream_chip(substream);

	pr_debug("setting buffer ptr param\n");
	intelhaddata->stream_info.period_elapsed = had_period_elapsed;
	intelhaddata->stream_info.had_substream = substream;
	intelhaddata->stream_info.buffer_ptr = 0;
	intelhaddata->stream_info.buffer_rendered = 0;
	intelhaddata->stream_info.sfreq = substream->runtime->rate;
	return 0;
}

/**
 * snd_intelhad_close- to free parameteres when stream is stopped
 *
 * @substream:  substream for which the function is called
 *
 * This function is called by ALSA framework when stream is stopped
 */
static int snd_intelhad_close(struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata;

	pr_debug("snd_intelhad_close called\n");
	intelhaddata = snd_pcm_substream_chip(substream);
	mutex_lock(&intelhaddata->had_lock);
	if (intelhaddata->playback_cnt)
		intelhaddata->playback_cnt--;

	intelhaddata->stream_info.buffer_rendered = 0;
	intelhaddata->stream_info.buffer_ptr = 0;
	intelhaddata->stream_info.str_id = 0;

	if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED)
		intelhaddata->drv_status = HAD_DRV_CONNECTED;
	mutex_unlock(&intelhaddata->had_lock);
	kfree(substream->runtime->private_data);
	return 0;
}

/**
 * snd_intelhad_hw_params- to setup the hardware parameters
 * like allocating the buffers
 *
 * @substream:  substream for which the function is called
 * @hw_params: hardware parameters
 *
 * This function is called by ALSA framework when hardware params are set
 */
static int snd_intelhad_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *hw_params)
{
	int retval;
	u32 buf_size;

	BUG_ON(!hw_params);
	pr_debug("snd_intelhad_hw_params called\n");

	buf_size = params_buffer_bytes(hw_params);

	if (buf_size % 64) {
		pr_err("Invalid buffer size\n");
		return -EINVAL;
	}

	retval = snd_pcm_lib_malloc_pages(substream, buf_size);
	if (retval < 0)
		return retval;
	pr_debug("allocated memory = %d\n", buf_size);
	memset(substream->runtime->dma_area, 0, buf_size);

	pr_debug("snd_intelhad_hw_params exited\n");
	return retval;
}

/**
 * snd_intelhad_hw_free- to release the resources allocated during
 * hardware params setup
 *
 * @substream:  substream for which the function is called
 *
 * This function is called by ALSA framework before close callback.
 *
 */
static int snd_intelhad_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("snd_intelhad_hw_free called\n");
	return snd_pcm_lib_free_pages(substream);
}

int snd_intelhad_configure_silence(struct snd_intelhad *intelhaddata)
{
	int retval = 0;
	u32 disp_samp_freq, n_param;

	pr_debug("Enter %s\n", __func__);
	/* Get N value in KHz */
	retval = intelhaddata->query_ops.hdmi_audio_get_caps(
				HAD_GET_SAMPLING_FREQ, &disp_samp_freq);
	if (retval) {
		pr_err("querying display sampling freq failed %#x\n", retval);
		goto out;
	} else
		pr_debug("%s:TMDS freq = %d kHz\n", __func__, disp_samp_freq);

	retval = snd_intelhad_prog_n(AUD_SAMPLE_RATE_44_1, &n_param,
								intelhaddata);
	if (retval) {
		pr_err("programming N value failed %#x\n", retval);
		goto out;
	}
	snd_intelhad_prog_cts(AUD_SAMPLE_RATE_44_1, disp_samp_freq, n_param,
			intelhaddata);

	snd_intelhad_prog_dip(NULL, intelhaddata, 1);
	retval = snd_intelhad_init_audio_ctrl(NULL, intelhaddata, 1);

out:
	return retval;
}

int snd_intelhad_start_silence(struct snd_intelhad *intelhaddata)
{
	int i, retval = 0;
	u32 buf_addr;

	pr_debug("Enter %s\n", __func__);

	buf_addr = virt_to_phys(intelhaddata->flat_data);

	/* Program buff C, D */
	for (i = HAD_BUF_TYPE_C; i < HAD_MAX_PERIODS; i++) {
		/* Program the buf registers with addr and len */
		intelhaddata->buf_info[i].buf_addr = buf_addr;
		intelhaddata->buf_info[i].buf_size = MAX_SZ_ZERO_BUF;

		intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
					buf_addr | BIT(0) | BIT(1));
		intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					MAX_SZ_ZERO_BUF);
		intelhaddata->buf_info[i].is_valid = true;
		pr_debug("buf[%d] addr=%#x  and size=%d\n", i,
				intelhaddata->buf_info[i].buf_addr,
				intelhaddata->buf_info[i].buf_size);
	}
	intelhaddata->valid_buf_cnt = HAD_MAX_PERIODS;
	intelhaddata->curr_buf = HAD_BUF_TYPE_C;

	intelhaddata->reg_ops.hdmi_audio_read_modify(AUD_CONFIG, 1,
				BIT(0));

	return retval;
}

int snd_intelhad_stop_silence(struct snd_intelhad *intelhaddata)
{
	int i, retval = 0;

	pr_debug("Enter %s\n", __func__);

	intelhaddata->reg_ops.hdmi_audio_read_modify(AUD_CONFIG, 0,
				BIT(0));

	/* Invalidate Audio buffers C & D */
	for (i = HAD_BUF_TYPE_C; i < HAD_MAX_PERIODS; i++) {
		/* Program the buf registers with addr and len */
		intelhaddata->reg_ops.hdmi_audio_write_register(
				AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
				0);
		intelhaddata->reg_ops.hdmi_audio_write_register(
				AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
				0);
	}
	/* Reset buffer pointers */
	intelhaddata->reg_ops.hdmi_audio_write_register(AUD_HDMI_STATUS, 1);
	intelhaddata->reg_ops.hdmi_audio_write_register(AUD_HDMI_STATUS, 0);

	return retval;
}

void snd_process_stop_trigger(struct snd_intelhad *intelhaddata)
{
	int buf_id, buff_done, i;
	u32 buf_addr;

	buff_done = intelhaddata->buff_done;
	if (intelhaddata->valid_buf_cnt-1 == buff_done)
		buf_id = HAD_BUF_TYPE_A;
	else
		buf_id = buff_done + 1;

	pr_debug("%s:buf_id=%d\n", __func__, buf_id);
	/* Invalidate Audio buffers */
	for (i = 0; i < HAD_MAX_PERIODS; i++) {
		if (i == buf_id)
			continue;
		if (i < HAD_BUF_TYPE_C) {
			/* invalidate */
			intelhaddata->buf_info[i].buf_size = 0;
			intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
					0);
			intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					0);
		} else {
			/* Program with silence buffer */
			buf_addr = virt_to_phys(intelhaddata->flat_data);

			/* Program the buf registers with addr and len */
			intelhaddata->buf_info[i].buf_addr = buf_addr;
			intelhaddata->buf_info[i].buf_size = MAX_SZ_ZERO_BUF;

			intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
					buf_addr | BIT(0) | BIT(1));
			intelhaddata->reg_ops.hdmi_audio_write_register(
					AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					(MAX_SZ_ZERO_BUF));
			intelhaddata->buf_info[i].is_valid = true;
			pr_debug("buf[%d] addr=%#x  and size=%d\n", i,
					intelhaddata->buf_info[i].buf_addr,
					intelhaddata->buf_info[i].buf_size);

		}
	}
}

/**
* snd_intelhad_pcm_trigger - stream activities are handled here
* @substream:substream for which the stream function is called
* @cmd:the stream commamd thats requested from upper layer
* This function is called whenever an a stream activity is invoked
*/
static int snd_intelhad_pcm_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	int retval = 0;
	int buf_id;
	unsigned long flag_irq;
	struct snd_intelhad *intelhaddata;
	struct had_stream_pvt *stream;
	struct hdmi_audio_registers_ops reg_ops;

	intelhaddata = snd_pcm_substream_chip(substream);
	stream = substream->runtime->private_data;
	reg_ops = intelhaddata->reg_ops;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("Trigger Start\n");
		stream->substream = substream;
		stream->stream_status = STREAM_RUNNING;
		snd_intelhad_read_len(intelhaddata);

		/* Disable local INTRs till register prgmng is done */
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irq);
		if (intelhaddata->buff_done == HAD_BUF_TYPE_D)
			buf_id = HAD_BUF_TYPE_C;
		else
			buf_id = HAD_BUF_TYPE_D;

		/* Re-program silence buffers */
		if (buf_id == HAD_BUF_TYPE_C) {
			/* Disable Buffer D*/
			reg_ops.hdmi_audio_write_register(AUD_BUF_A_ADDR +
					(HAD_BUF_TYPE_D * HAD_REG_WIDTH), 0);
			reg_ops.hdmi_audio_write_register(AUD_BUF_A_LENGTH +
					(HAD_BUF_TYPE_D * HAD_REG_WIDTH), 0);
		}
		retval = snd_intelhad_prog_buffer(intelhaddata, HAD_BUF_TYPE_A,
				HAD_BUF_TYPE_B);
		/* Start reporting BUFFER_DONE/UNDERRUN to above layers*/
		intelhaddata->pcm_active = 1;
		intelhaddata->start_trigger = 1;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irq);
		pr_debug("Processed _Start, buf_id = %d\n", buf_id);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("Trigger Stop\n");
		snd_intelhad_read_len(intelhaddata);
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irq);
		intelhaddata->stream_info.str_id = 0;
		intelhaddata->send_data = 0;

		/* Stop reporting BUFFER_DONE/UNDERRUN to above layers*/
		intelhaddata->pcm_active = 0;
		intelhaddata->stop_trigger = 1;
		/* Send zero filled data */
		if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED)
			snd_process_stop_trigger(intelhaddata);
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irq);
		cancel_delayed_work(&intelhaddata->dummy_audio);

		stream->stream_status = STREAM_DROPPED;
		break;

	default:
		retval = -EINVAL;
	}
	return retval;
}

/**
* snd_intelhad_pcm_prepare- internal preparation before starting a stream
*
* @substream:  substream for which the function is called
*
* This function is called when a stream is started for internal preparation.
*/
static int snd_intelhad_pcm_prepare(struct snd_pcm_substream *substream)
{
	int retval;
	u32 disp_samp_freq, n_param;
	struct snd_intelhad *intelhaddata;
	struct snd_pcm_runtime *runtime;

	pr_debug("pcm_prepare called\n");

	runtime = substream->runtime;
	pr_debug("period_size=%d\n",
				frames_to_bytes(runtime, runtime->period_size));
	pr_debug("periods=%d\n", runtime->periods);
	pr_debug("buffer_size=%d\n", snd_pcm_lib_buffer_bytes(substream));
	pr_debug("rate=%d\n", runtime->rate);
	pr_debug("channels=%d\n", runtime->channels);

	intelhaddata = snd_pcm_substream_chip(substream);
	if (intelhaddata->stream_info.str_id) {
		pr_debug("_prepare is called for existing str_id#%d\n",
					intelhaddata->stream_info.str_id);
		retval = snd_intelhad_pcm_trigger(substream,
						SNDRV_PCM_TRIGGER_STOP);
		return retval;
	}

	intelhaddata->stream_info.str_id = intelhaddata->playback_cnt;
	snprintf(substream->pcm->id, sizeof(substream->pcm->id),
			"%d", intelhaddata->stream_info.str_id);
	retval = snd_intelhad_init_stream(substream);
	if (retval)
		goto prep_end;
	/* Get N value in KHz */
	retval = intelhaddata->query_ops.hdmi_audio_get_caps(
				HAD_GET_SAMPLING_FREQ, &disp_samp_freq);
	if (retval) {
		pr_err("querying display sampling freq failed %#x\n", retval);
		goto prep_end;
	} else
		pr_debug("TMDS freq = %d kHz\n", disp_samp_freq);

	retval = snd_intelhad_prog_n(substream->runtime->rate, &n_param,
								intelhaddata);
	if (retval) {
		pr_err("programming N value failed %#x\n", retval);
		goto prep_end;
	}
	snd_intelhad_prog_cts(substream->runtime->rate,
					disp_samp_freq, n_param, intelhaddata);

	snd_intelhad_prog_dip(substream, intelhaddata, 0);

prep_end:
	return retval;
}

/**
 * snd_intelhad_pcm_pointer- to send the current buffer pointerprocessed by hw
 *
 * @substream:  substream for which the function is called
 *
 * This function is called by ALSA framework to get the current hw buffer ptr
 * when a period is elapsed
 */
static snd_pcm_uframes_t snd_intelhad_pcm_pointer(
					struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata;
	u32 bytes_rendered;

	intelhaddata = snd_pcm_substream_chip(substream);

	if (intelhaddata->flag_underrun) {
		intelhaddata->flag_underrun = 0;
		return SNDRV_PCM_POS_XRUN;
	}

	div_u64_rem(intelhaddata->stream_info.buffer_rendered,
			intelhaddata->stream_info.ring_buf_size,
			&(bytes_rendered));
	intelhaddata->stream_info.buffer_ptr = bytes_to_frames(
						substream->runtime,
						bytes_rendered);
	return intelhaddata->stream_info.buffer_ptr;
}

int hdmi_audio_mode_change(struct snd_pcm_substream *substream)
{
	int retval = 0;
	u32 disp_samp_freq, n_param;
	struct snd_intelhad *intelhaddata;

	intelhaddata = snd_pcm_substream_chip(substream);

	/* Disable Audio */
	intelhaddata->reg_ops.hdmi_audio_read_modify(
			AUD_CONFIG, 0, BIT(0));

	/* Update CTS value */
	retval = intelhaddata->query_ops.hdmi_audio_get_caps(
				HAD_GET_SAMPLING_FREQ, &disp_samp_freq);
	if (retval) {
		pr_err("querying display sampling freq failed %#x\n", retval);
		goto out;
	} else
		pr_debug("TMDS freq = %d kHz\n", disp_samp_freq);

	retval = snd_intelhad_prog_n(substream->runtime->rate, &n_param,
								intelhaddata);
	if (retval) {
		pr_err("programming N value failed %#x\n", retval);
		goto out;
	}
	snd_intelhad_prog_cts(substream->runtime->rate,
					disp_samp_freq, n_param, intelhaddata);

	/* Enable Audio */
	intelhaddata->reg_ops.hdmi_audio_read_modify(
			AUD_CONFIG, 1, BIT(0));

out:
	return retval;
}

void dummy_audio_play(struct work_struct *work)
{
	struct snd_intelhad *intelhaddata = had_pvt_data;

	mutex_lock(&intelhaddata->had_lock);
	if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED) {
		mutex_unlock(&intelhaddata->had_lock);
		pr_debug("HDMI device still connected\n");
		return;
	}

	if (!intelhaddata->send_data) {
		mutex_unlock(&intelhaddata->had_lock);
		pr_debug("HDMI device_flag is reset\n");
		return;
	}

	had_event_handler(HAD_EVENT_AUDIO_BUFFER_DONE, intelhaddata);
	mutex_unlock(&intelhaddata->had_lock);
	schedule_delayed_work(&intelhaddata->dummy_audio, intelhaddata->timer);
}

/*PCM operations structure and the calls back for the same */
struct snd_pcm_ops snd_intelhad_playback_ops = {
	.open =		snd_intelhad_open,
	.close =	snd_intelhad_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_intelhad_hw_params,
	.hw_free =	snd_intelhad_hw_free,
	.prepare =	snd_intelhad_pcm_prepare,
	.trigger =	snd_intelhad_pcm_trigger,
	.pointer =	snd_intelhad_pcm_pointer,
};

/**
 * snd_intelhad_create - to crete alsa card instance
 *
 * @intelhaddata: pointer to internal context
 * @card: pointer to card
 *
 * This function is called when the hdmi cable is plugged in
 */
static int __devinit snd_intelhad_create(
		struct snd_intelhad *intelhaddata,
		struct snd_card *card)
{
	int retval;
	static struct snd_device_ops ops = {
	};

	BUG_ON(!intelhaddata);
	/* ALSA api to register the device */
	retval = snd_device_new(card, SNDRV_DEV_LOWLEVEL, intelhaddata, &ops);
	return retval;
}
/**
 * snd_intelhad_pcm_free - to free the memory allocated
 *
 * @pcm: pointer to pcm instance
 * This function is called when the device is removed
 */
static void snd_intelhad_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("Freeing PCM preallocated pages\n");
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static int had_iec958_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

static int had_iec958_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_intelhad *intelhaddata = snd_kcontrol_chip(kcontrol);
	mutex_lock(&intelhaddata->had_lock);
	ucontrol->value.iec958.status[0] = (intelhaddata->aes_bits >> 0) & 0xff;
	ucontrol->value.iec958.status[1] = (intelhaddata->aes_bits >> 8) & 0xff;
	ucontrol->value.iec958.status[2] =
					(intelhaddata->aes_bits >> 16) & 0xff;
	ucontrol->value.iec958.status[3] =
					(intelhaddata->aes_bits >> 24) & 0xff;
	mutex_unlock(&intelhaddata->had_lock);
	return 0;
}
static int had_iec958_mask_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.iec958.status[0] = 0xff;
	ucontrol->value.iec958.status[1] = 0xff;
	ucontrol->value.iec958.status[2] = 0xff;
	ucontrol->value.iec958.status[3] = 0xff;
	return 0;
}
static int had_iec958_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	unsigned int val;
	struct snd_intelhad *intelhaddata = snd_kcontrol_chip(kcontrol);

	pr_debug("entered had_iec958_put\n");
	val = (ucontrol->value.iec958.status[0] << 0) |
		(ucontrol->value.iec958.status[1] << 8) |
		(ucontrol->value.iec958.status[2] << 16) |
		(ucontrol->value.iec958.status[3] << 24);
	mutex_lock(&intelhaddata->had_lock);
	if (intelhaddata->aes_bits != val) {
		intelhaddata->aes_bits = val;
		mutex_unlock(&intelhaddata->had_lock);
		return 1;
	}
	mutex_unlock(&intelhaddata->had_lock);
	return 1;
}

static struct snd_kcontrol_new had_control_iec958_mask = {
	.access =   SNDRV_CTL_ELEM_ACCESS_READ,
	.iface =    SNDRV_CTL_ELEM_IFACE_PCM,
	.name =     SNDRV_CTL_NAME_IEC958("", PLAYBACK, MASK),
	.info =     had_iec958_info, /* shared */
	.get =      had_iec958_mask_get,
};

static struct snd_kcontrol_new had_control_iec958 = {
	.iface =    SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
	.info =         had_iec958_info,
	.get =          had_iec958_get,
	.put =          had_iec958_put
};

static struct snd_intel_had_interface had_interface = {
	.name =         "hdmi-audio",
	.suspend =      hdmi_audio_suspend,
	.resume =       hdmi_audio_resume,
};

/**
 * hdmi_audio_probe - to create sound card instance for HDMI audio playabck
 *
 *@haddata: pointer to HAD private data
 *@card_id: card for which probe is called
 *
 * This function is called when the hdmi cable is plugged in. This function
 * creates and registers the sound card with ALSA
 */
static int __devinit hdmi_audio_probe(struct platform_device *devptr)
{

	int retval;
	struct snd_pcm *pcm;
	struct snd_card *card;
	struct had_callback_ops ops_cb;
	struct snd_intelhad *intelhaddata;

	pr_debug("Enter %s\n", __func__);

	/* allocate memory for saving internal context and working */
	intelhaddata = kzalloc(sizeof(*intelhaddata), GFP_KERNEL);
	if (!intelhaddata) {
		pr_err("mem alloc failed\n");
		return -ENOMEM;
	}

	had_pvt_data = intelhaddata;
	ops_cb.intel_had_event_call_back = had_event_handler;

	/* registering with display driver to get access to display APIs */

	retval = intel_hdmi_audio_query_capabilities(
			ops_cb.intel_had_event_call_back,
			&(intelhaddata->reg_ops),
			&(intelhaddata->query_ops));
	if (retval) {
		pr_err("querying display driver APIs failed %#x\n", retval);
		goto free_haddata;
	}
	mutex_lock(&had_mutex);
	mutex_init(&intelhaddata->had_lock);
	spin_lock_init(&intelhaddata->had_spinlock);
	intelhaddata->drv_status = HAD_DRV_DISCONNECTED;
	/* create a card instance with ALSA framework */
	retval = snd_card_create(hdmi_card_index, hdmi_card_id,
				THIS_MODULE, 0, &card);
	if (retval)
		goto unlock_mutex;
	intelhaddata->card = card;
	intelhaddata->card_id = hdmi_card_id;
	intelhaddata->card_index = card->number;
	intelhaddata->playback_cnt = 0;
	intelhaddata->flag_underrun = 0;
	intelhaddata->aes_bits = SNDRV_PCM_DEFAULT_CON_SPDIF;
	strncpy(card->driver, INTEL_HAD, strlen(INTEL_HAD));
	strncpy(card->shortname, INTEL_HAD, strlen(INTEL_HAD));

	retval = snd_pcm_new(card, INTEL_HAD, PCM_INDEX, MAX_PB_STREAMS,
						MAX_CAP_STREAMS, &pcm);
	if (retval)
		goto err;

	/* setup private data which can be retrieved when required */
	pcm->private_data = intelhaddata;
	pcm->private_free = snd_intelhad_pcm_free;
	pcm->info_flags = 0;
	strncpy(pcm->name, card->shortname, strlen(card->shortname));
	/* setup the ops for palyabck */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			    &snd_intelhad_playback_ops);
	/* allocate dma pages for ALSA stream operations */
	retval = snd_pcm_lib_preallocate_pages_for_all(pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			HAD_MIN_BUFFER, HAD_MAX_BUFFER);
	if (retval)
		goto err;

	/* internal function call to register device with ALSA */
	retval = snd_intelhad_create(intelhaddata, card);
	if (retval)
		goto err;

	card->private_data = &intelhaddata;
	retval = snd_card_register(card);
	if (retval)
		goto err;

	/* IEC958 controls */
	retval = snd_ctl_add(card, snd_ctl_new1(&had_control_iec958_mask,
						intelhaddata));
	if (retval < 0)
		goto err;
	retval = snd_ctl_add(card, snd_ctl_new1(&had_control_iec958,
						intelhaddata));
	if (retval < 0)
		goto err;

	/* Initialize dummy audio workqueue */
	INIT_DELAYED_WORK(&intelhaddata->dummy_audio, dummy_audio_play);

	/* Allocate memory for flat data */
	intelhaddata->flat_data = kzalloc((MAX_SZ_ZERO_BUF), GFP_KERNEL);
	if (!intelhaddata->flat_data) {
		retval = -ENOMEM;
		goto err;
	}

	mutex_unlock(&had_mutex);
	retval = display_register(&had_interface, intelhaddata);
	if (retval) {
		pr_err("registering with display driver failed %#x\n", retval);
		snd_card_free(card);
		goto free_haddata;
	}

	return retval;
err:
	pr_err("Error returned from %s api %#x\n", __func__, retval);
	snd_card_free(card);
unlock_mutex:
	mutex_unlock(&had_mutex);
free_haddata:
	kfree(intelhaddata);
	intelhaddata = NULL;
	return retval;
}

/**
 * hdmi_audio_remove - removes the alsa card
 *
 *@haddata: pointer to HAD private data
 *
 * This function is called when the hdmi cable is un-plugged. This function
 * free the sound card.
 */
static int __devexit hdmi_audio_remove(struct platform_device *devptr)
{
	struct snd_intelhad *intelhaddata = had_pvt_data;
	int caps;

	pr_debug("Enter %s\n", __func__);

	if (!intelhaddata)
		return 0;

	mutex_lock(&intelhaddata->had_lock);
	if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED) {
		caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		intelhaddata->query_ops.hdmi_audio_set_caps(
				HAD_SET_DISABLE_AUDIO_INT, &caps);
		intelhaddata->query_ops.hdmi_audio_set_caps(
				HAD_SET_DISABLE_AUDIO, NULL);
		snd_intelhad_stop_silence(intelhaddata);
	}
	kfree(intelhaddata->flat_data);
	snd_card_free(intelhaddata->card);
	mutex_unlock(&intelhaddata->had_lock);
	kfree(intelhaddata);
	return 0;
}

static struct platform_driver had_driver = {
	.probe =        hdmi_audio_probe,
	.remove		= __devexit_p(hdmi_audio_remove),
	.suspend =      NULL,
	.resume =       NULL,
	.driver		= {
		.name	= HDMI_AUDIO_DRIVER
	},
};

/*
* alsa_card_intelhad_init- driver init function
* This function is called when driver module is inserted
*/
static int __init alsa_card_intelhad_init(void)
{
	int retval;

	pr_debug("Enter %s\n", __func__);

	pr_info("******** HAD DRIVER loading.. Ver: %s\n",
					HAD_DRIVER_VERSION);

	retval = platform_driver_register(&had_driver);
	if (retval < 0) {
		pr_err("Platform driver register failed\n");
		return retval;
	}

	pr_debug("init complete\n");
	return retval;
}

/**
* alsa_card_intelhad_exit- driver exit function
* This function is called when driver module is removed
*/
static void __exit alsa_card_intelhad_exit(void)
{
	pr_debug("had_exit called\n");
	platform_driver_unregister(&had_driver);
}
late_initcall(alsa_card_intelhad_init);
module_exit(alsa_card_intelhad_exit);
