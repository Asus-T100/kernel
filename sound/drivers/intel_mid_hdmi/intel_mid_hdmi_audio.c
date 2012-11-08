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

#include <psb_powermgmt.h>

#define PCM_INDEX		0
#define MAX_PB_STREAMS		1
#define MAX_CAP_STREAMS		0
#define HDMI_AUDIO_DRIVER	"hdmi-audio"
static DEFINE_MUTEX(had_mutex);

/*standard module options for ALSA. This module supports only one card*/
static int hdmi_card_index = SNDRV_DEFAULT_IDX1;
static char *hdmi_card_id = SNDRV_DEFAULT_STR1;
static struct snd_intelhad *had_data;

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
#define SWAP_LFE_CENTER		0x00fac4c8

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
	.buffer_bytes_max = HAD_MAX_BUFFER,
	.period_bytes_min = HAD_MIN_PERIOD_BYTES,
	.period_bytes_max = HAD_MAX_PERIOD_BYTES,
	.periods_min = HAD_MIN_PERIODS,
	.periods_max = HAD_MAX_PERIODS,
	.fifo_size = HAD_FIFO_SIZE,
};

/* Register access functions */

inline int had_get_hwstate(struct snd_intelhad *intelhaddata)
{
	/* Check for device presence -SW state */
	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED) {
		pr_debug("%s:Device not connected:%d\n", __func__,
				intelhaddata->drv_status);
		return -ENODEV;
	}

	/* Check for device presence -HW state */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_ONLY_IF_ON)) {
		pr_err("%s:Device not connected\n", __func__);
		dump_stack();
		/* HOT_UNPLUG event can be sent to
		 * maintain correct state within HAD
		 * had_event_handler(HAD_EVENT_HOT_UNPLUG, intelhaddata);
		 * Drop all acuired locks before executing this.
		 */
		return -ENODEV;
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}

inline int had_get_caps(enum had_caps_list query, void *caps)
{
	int retval;
	struct snd_intelhad *intelhaddata = had_data;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = intelhaddata->query_ops.hdmi_audio_get_caps(query,
				caps);

	return retval;
}

inline int had_set_caps(enum had_caps_list set_element , void *caps)
{
	int retval;
	struct snd_intelhad *intelhaddata = had_data;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = intelhaddata->query_ops.hdmi_audio_set_caps(
				set_element, caps);

	return retval;
}

inline int had_read_register(uint32_t reg_addr, uint32_t *data)
{
	int retval;
	struct snd_intelhad *intelhaddata = had_data;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = intelhaddata->reg_ops.hdmi_audio_read_register(
				reg_addr, data);

	return retval;
}

inline int had_write_register(uint32_t reg_addr, uint32_t data)
{
	int retval;
	struct snd_intelhad *intelhaddata = had_data;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = intelhaddata->reg_ops.hdmi_audio_write_register(
				reg_addr, data);

	return retval;
}

inline int had_read_modify(uint32_t reg_addr, uint32_t data, uint32_t mask)
{
	int retval;
	struct snd_intelhad *intelhaddata = had_data;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = intelhaddata->reg_ops.hdmi_audio_read_modify(
				reg_addr, data, mask);

	return retval;
}

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
	int format;

	ch_stat0.status_0_regx.lpcm_id = (intelhaddata->aes_bits &
						IEC958_AES0_NONAUDIO)>>1;
	ch_stat0.status_0_regx.clk_acc = (intelhaddata->aes_bits &
						IEC958_AES3_CON_CLOCK)>>4;
	switch (substream->runtime->rate) {
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
	had_write_register(AUD_CH_STATUS_0, ch_stat0.status_0_regval);

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
	had_write_register(AUD_CH_STATUS_1, ch_stat1.status_1_regval);

	buf_cfg.buf_cfg_regx.fifo_width = FIFO_THRESHOLD;
	buf_cfg.buf_cfg_regx.aud_delay = 0;
	had_write_register(AUD_BUF_CONFIG, buf_cfg.buf_cfgval);

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
	had_write_register(AUD_CONFIG, cfg_val.cfg_regval);
	return 0;
}

/*
 * Compute derived values in channel_allocations[].
 */
static void init_channel_allocations(void)
{
	int i, j;
	struct cea_channel_speaker_allocation *p;

	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		p = channel_allocations + i;
		p->channels = 0;
		p->spk_mask = 0;
		for (j = 0; j < ARRAY_SIZE(p->speakers); j++)
			if (p->speakers[j]) {
				p->channels++;
				p->spk_mask |= p->speakers[j];
			}
	}
}

/*
 * The transformation takes two steps:
 *
 *      eld->spk_alloc => (eld_speaker_allocation_bits[]) => spk_mask
 *            spk_mask => (channel_allocations[])         => ai->CA
 *
 * TODO: it could select the wrong CA from multiple candidates.
*/
static int snd_intelhad_channel_allocation(struct snd_intelhad *intelhaddata,
					int channels)
{
	int i;
	int ca = 0;
	int spk_mask = 0;

	init_channel_allocations();
	/*
	* CA defaults to 0 for basic stereo audio
	*/
	if (channels <= 2)
		return 0;

	/*
	* expand ELD's speaker allocation mask
	*
	* ELD tells the speaker mask in a compact(paired) form,
	* expand ELD's notions to match the ones used by Audio InfoFrame.
	*/
	for (i = 0; i < ARRAY_SIZE(eld_speaker_allocation_bits); i++) {
		if (intelhaddata->eeld.speaker_allocation_block & (1 << i))
				spk_mask |= eld_speaker_allocation_bits[i];
	}

	/* search for the first working match in the CA table */
	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		if (channels == channel_allocations[i].channels &&
		(spk_mask & channel_allocations[i].spk_mask) ==
				channel_allocations[i].spk_mask) {
			ca = channel_allocations[i].ca_index;
			break;
		}
	}

	pr_debug("HDMI: select CA 0x%x for %d\n", ca, channels);

	return ca;
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
	int channels;
	channels = substream->runtime->channels;

	had_write_register(AUD_CNTL_ST, ctrl_state.ctrl_val);

	frame2.fr2_regx.chnl_cnt = substream->runtime->channels - 1;

	frame3.fr3_regx.chnl_alloc = snd_intelhad_channel_allocation(
					intelhaddata, channels);

	/*Calculte the byte wide checksum for all valid DIP words*/
	for (i = 0; i < BYTES_PER_WORD; i++)
		checksum += (INFO_FRAME_WORD1 >> i*BITS_PER_BYTE) & MASK_BYTE0;
	for (i = 0; i < BYTES_PER_WORD; i++)
		checksum += (frame2.fr2_val >> i*BITS_PER_BYTE) & MASK_BYTE0;
	for (i = 0; i < BYTES_PER_WORD; i++)
		checksum += (frame3.fr3_val >> i*BITS_PER_BYTE) & MASK_BYTE0;

	frame2.fr2_regx.chksum = -(checksum);

	had_write_register(AUD_HDMIW_INFOFR, INFO_FRAME_WORD1);
	had_write_register(AUD_HDMIW_INFOFR, frame2.fr2_val);
	had_write_register(AUD_HDMIW_INFOFR, frame3.fr3_val);

	/* program remaining DIP words with zero */
	for (i = 0; i < HAD_MAX_DIP_WORDS-VALID_DIP_WORDS; i++)
		had_write_register(AUD_HDMIW_INFOFR, 0x0);

	ctrl_state.ctrl_regx.dip_freq = 1;
	ctrl_state.ctrl_regx.dip_en_sta = 1;
	had_write_register(AUD_CNTL_ST, ctrl_state.ctrl_val);
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
	if (!substream) {
		pr_err("substream is NULL\n");
		dump_stack();
		return 0;
	}

	ring_buf_addr = substream->runtime->dma_addr;
	ring_buf_size = snd_pcm_lib_buffer_bytes(substream);
	intelhaddata->stream_info.ring_buf_size = ring_buf_size;
	period_bytes = frames_to_bytes(substream->runtime,
				substream->runtime->period_size);
	num_periods = substream->runtime->periods;

	/* buffer addr should  be 64 byte aligned, period bytes
	 will be used to calculate addr offset*/
	period_bytes &= ~0x3F;

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

		had_write_register(AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
					intelhaddata->buf_info[i].buf_addr |
					BIT(0) | BIT(1));
		had_write_register(AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					period_bytes);
		intelhaddata->buf_info[i].is_valid = true;
	}
	pr_debug("%s:buf[%d-%d] addr=%#x  and size=%d\n", __func__, start, end,
			intelhaddata->buf_info[start].buf_addr,
			intelhaddata->buf_info[start].buf_size);
	intelhaddata->valid_buf_cnt = num_periods;
	return 0;
}

inline int snd_intelhad_read_len(struct snd_intelhad *intelhaddata)
{
	int i, retval = 0;
	u32 len[4];

	for (i = 0; i < 4 ; i++) {
		had_read_register(AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					&len[i]);
		if (!len[i])
			retval++;
	}
	if (retval != 1) {
		for (i = 0; i < 4 ; i++)
			pr_debug("buf[%d] size=%d\n", i, len[i]);
	}

	return retval;
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
	pr_debug("TMDS value=%d, N value=%d, CTS Value=%d\n",
			tmds, n_param, cts_val);
	had_write_register(AUD_HDMI_CTS, (BIT(20) | cts_val));
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
	had_write_register(AUD_N_ENABLE, (BIT(20) | n_val));
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
	struct had_pvt_data *had_stream;
	int retval;

	pr_debug("snd_intelhad_open called\n");
	intelhaddata = snd_pcm_substream_chip(substream);
	had_stream = intelhaddata->private_data;

	pm_runtime_get(intelhaddata->dev);

	/*
	 * HDMI driver might suspend the device already,
	 * so we return it on
	 */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
		pr_err("HDMI device can't be turned on\n");
		retval = -ENODEV;
		goto exit_put_handle;
	}

	if (had_get_hwstate(intelhaddata)) {
		pr_err("%s: HDMI cable plugged-out\n", __func__);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		retval = -ENODEV;
		goto exit_put_handle;
	}
	runtime = substream->runtime;

	/* Check, if device already in use */
	if (runtime->private_data) {
		pr_err("Device already in use\n");
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		retval = -EBUSY;
		goto exit_put_handle;
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

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

	/* Make sure, that the period size is always aligned
	 * 64byte boundary
	 */
	retval = snd_pcm_hw_constraint_step(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 64);
	if (retval < 0) {
		pr_err("%s:step_size=64 failed,err=%d\n", __func__, retval);
		kfree(stream);
		goto exit_err;
	}

	return retval;
exit_err:
	runtime->private_data = NULL;
exit_put_handle:
	pm_runtime_put(intelhaddata->dev);
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
	struct snd_pcm_runtime *runtime;

	pr_debug("snd_intelhad_close called\n");
	intelhaddata = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;

	intelhaddata->stream_info.buffer_rendered = 0;
	intelhaddata->stream_info.buffer_ptr = 0;
	intelhaddata->stream_info.str_id = 0;
	intelhaddata->stream_info.had_substream = NULL;

	/* Check if following drv_status modification is required - VA */
	if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED)
		intelhaddata->drv_status = HAD_DRV_CONNECTED;
	kfree(runtime->private_data);
	runtime->private_data = NULL;
	pm_runtime_put(intelhaddata->dev);
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
	unsigned long addr;
	int pages, buf_size, retval;

	BUG_ON(!hw_params);

	buf_size = params_buffer_bytes(hw_params);
	retval = snd_pcm_lib_malloc_pages(substream, buf_size);
	if (retval < 0)
		return retval;
	pr_debug("%s:allocated memory = %d\n", __func__, buf_size);
	/* mark the pages as uncached region */
	addr = (unsigned long) substream->runtime->dma_area;
	pages = (substream->runtime->dma_bytes + PAGE_SIZE - 1) / PAGE_SIZE;
	retval = set_memory_uc(addr, pages);
	if (retval) {
		pr_err("set_memory_uc failed.Error:%d\n", retval);
		return retval;
	}
	memset(substream->runtime->dma_area, 0, buf_size);

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
	unsigned long addr;
	u32 pages;

	pr_debug("snd_intelhad_hw_free called\n");

	/* mark back the pages as cached/writeback region before the free */
	if (substream->runtime->dma_area != NULL) {
		addr = (unsigned long) substream->runtime->dma_area;
		pages = (substream->runtime->dma_bytes + PAGE_SIZE - 1) /
								PAGE_SIZE;
		set_memory_wb(addr, pages);
		return snd_pcm_lib_free_pages(substream);
	}
	return 0;
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
	int caps, retval = 0;
	unsigned long flag_irq;
	struct snd_intelhad *intelhaddata;
	struct had_stream_pvt *stream;
	struct had_pvt_data *had_stream;

	intelhaddata = snd_pcm_substream_chip(substream);
	stream = substream->runtime->private_data;
	had_stream = intelhaddata->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("Trigger Start\n");

		/* Disable local INTRs till register prgmng is done */
		if (had_get_hwstate(intelhaddata)) {
			pr_err("_START: HDMI cable plugged-out\n");
			retval = -ENODEV;
			break;
		}
		stream->stream_status = STREAM_RUNNING;

		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irq);
		had_stream->stream_type = HAD_RUNNING_STREAM;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irq);

		/* Enable Audio */
		/* ToDo: Need to enable UNDERRUN interrupts as well
		   caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		   */
		caps = HDMI_AUDIO_BUFFER_DONE;
		retval = had_set_caps(HAD_SET_ENABLE_AUDIO_INT, &caps);
		retval = had_set_caps(HAD_SET_ENABLE_AUDIO, NULL);
		had_read_modify(AUD_CONFIG, 1, BIT(0));

		pr_debug("Processed _Start\n");

		break;

	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("Trigger Stop\n");
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irq);
		intelhaddata->stream_info.str_id = 0;
		intelhaddata->curr_buf = 0;

		/* Stop reporting BUFFER_DONE/UNDERRUN to above layers*/

		had_stream->stream_type = HAD_INIT;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irq);
		/* Disable Audio */
		/* ToDo: Need to disable UNDERRUN interrupts as well
		   caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		   */
		caps = HDMI_AUDIO_BUFFER_DONE;
		had_set_caps(HAD_SET_DISABLE_AUDIO_INT, &caps);
		had_set_caps(HAD_SET_DISABLE_AUDIO, NULL);
		had_read_modify(AUD_CONFIG, 0, BIT(0));
		/* Reset buffer pointers */
		had_write_register(AUD_HDMI_STATUS, 1);
		had_write_register(AUD_HDMI_STATUS, 0);
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
	struct had_pvt_data *had_stream;

	pr_debug("pcm_prepare called\n");

	intelhaddata = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	had_stream = intelhaddata->private_data;

	if (had_get_hwstate(intelhaddata)) {
		pr_err("%s: HDMI cable plugged-out\n", __func__);
		snd_pcm_stop(substream, SNDRV_PCM_STATE_DISCONNECTED);
		retval = -ENODEV;
		goto prep_end;
	}

	pr_debug("period_size=%d\n",
				frames_to_bytes(runtime, runtime->period_size));
	pr_debug("periods=%d\n", runtime->periods);
	pr_debug("buffer_size=%d\n", snd_pcm_lib_buffer_bytes(substream));
	pr_debug("rate=%d\n", runtime->rate);
	pr_debug("channels=%d\n", runtime->channels);

	if (intelhaddata->stream_info.str_id) {
		pr_debug("_prepare is called for existing str_id#%d\n",
					intelhaddata->stream_info.str_id);
		retval = snd_intelhad_pcm_trigger(substream,
						SNDRV_PCM_TRIGGER_STOP);
		return retval;
	}

	retval = snd_intelhad_init_stream(substream);
	if (retval)
		goto prep_end;


	/* Get N value in KHz */
	retval = had_get_caps(HAD_GET_SAMPLING_FREQ, &disp_samp_freq);
	if (retval) {
		pr_err("querying display sampling freq failed %#x\n", retval);
		goto prep_end;
	}

	had_get_caps(HAD_GET_ELD, &intelhaddata->eeld);

	retval = snd_intelhad_prog_n(substream->runtime->rate, &n_param,
								intelhaddata);
	if (retval) {
		pr_err("programming N value failed %#x\n", retval);
		goto prep_end;
	}
	snd_intelhad_prog_cts(substream->runtime->rate,
					disp_samp_freq, n_param, intelhaddata);

	snd_intelhad_prog_dip(substream, intelhaddata, 0);

	retval = snd_intelhad_init_audio_ctrl(substream, intelhaddata, 0);

	/* Prog buffer address */
	retval = snd_intelhad_prog_buffer(intelhaddata,
			HAD_BUF_TYPE_A, HAD_BUF_TYPE_D);

	/* Program channel mapping in following order:
	   FL, FR, C, LFE, RL, RR */

	had_write_register(AUD_BUF_CH_SWAP, SWAP_LFE_CENTER);

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
	u32 bytes_rendered = 0;

	intelhaddata = snd_pcm_substream_chip(substream);

	if (intelhaddata->flag_underrun) {
		intelhaddata->flag_underrun = 0;
		return SNDRV_PCM_POS_XRUN;
	}

	if (intelhaddata->stream_info.buffer_rendered)
		div_u64_rem(intelhaddata->stream_info.buffer_rendered,
			intelhaddata->stream_info.ring_buf_size,
			&(bytes_rendered));

	intelhaddata->stream_info.buffer_ptr = bytes_to_frames(
						substream->runtime,
						bytes_rendered);
	return intelhaddata->stream_info.buffer_ptr;
}

/**
* snd_intelhad_pcm_mmap- mmaps a kernel buffer to user space for copying data
*
* @substream:  substream for which the function is called
* @vma:		struct instance of memory VMM memory area
*
* This function is called by OS when a user space component
* tries to get mmap memory from driver
*/
static int snd_intelhad_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{

	pr_debug("entry with prot:%s\n", __func__);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			substream->dma_buffer.addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

int hdmi_audio_mode_change(struct snd_pcm_substream *substream)
{
	int retval = 0;
	u32 disp_samp_freq, n_param;
	struct snd_intelhad *intelhaddata;

	intelhaddata = snd_pcm_substream_chip(substream);

	/* Disable Audio */
	had_read_modify(AUD_CONFIG, 0, BIT(0));

	/* Update CTS value */
	retval = had_get_caps(HAD_GET_SAMPLING_FREQ, &disp_samp_freq);
	if (retval) {
		pr_err("querying display sampling freq failed %#x\n", retval);
		goto out;
	}

	retval = snd_intelhad_prog_n(substream->runtime->rate, &n_param,
								intelhaddata);
	if (retval) {
		pr_err("programming N value failed %#x\n", retval);
		goto out;
	}
	snd_intelhad_prog_cts(substream->runtime->rate,
					disp_samp_freq, n_param, intelhaddata);

	/* Enable Audio */
	had_read_modify(AUD_CONFIG, 1, BIT(0));

out:
	return retval;
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
	.mmap =	snd_intelhad_pcm_mmap,
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
	ucontrol->value.iec958.status[0] = (intelhaddata->aes_bits >> 0) & 0xff;
	ucontrol->value.iec958.status[1] = (intelhaddata->aes_bits >> 8) & 0xff;
	ucontrol->value.iec958.status[2] =
					(intelhaddata->aes_bits >> 16) & 0xff;
	ucontrol->value.iec958.status[3] =
					(intelhaddata->aes_bits >> 24) & 0xff;
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
	if (intelhaddata->aes_bits != val) {
		intelhaddata->aes_bits = val;
		return 1;
	}
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
	.query =        hdmi_audio_query,
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
	struct had_pvt_data *had_stream;

	pr_debug("Enter %s\n", __func__);

	/* allocate memory for saving internal context and working */
	intelhaddata = kzalloc(sizeof(*intelhaddata), GFP_KERNEL);
	if (!intelhaddata) {
		pr_err("mem alloc failed\n");
		return -ENOMEM;
	}

	had_stream = kzalloc(sizeof(*had_stream), GFP_KERNEL);
	if (!had_stream) {
		pr_err("mem alloc failed\n");
		retval = -ENOMEM;
		goto free_haddata;
	}

	had_data = intelhaddata;
	ops_cb.intel_had_event_call_back = had_event_handler;

	/* registering with display driver to get access to display APIs */

	retval = mid_hdmi_audio_setup(
			ops_cb.intel_had_event_call_back,
			&(intelhaddata->reg_ops),
			&(intelhaddata->query_ops));
	if (retval) {
		pr_err("querying display driver APIs failed %#x\n", retval);
		goto free_hadstream;
	}
	mutex_lock(&had_mutex);
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
	intelhaddata->private_data = had_stream;
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
	/* allocate dma pages for ALSA stream operations
	 * memory allocated is based on size, not max value
	 * thus using same argument for max & size
	 */
	retval = snd_pcm_lib_preallocate_pages_for_all(pcm,
			SNDRV_DMA_TYPE_DEV, card->dev,
			HAD_MAX_BUFFER, HAD_MAX_BUFFER);
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

	/* Allocate memory for flat data */
	intelhaddata->flat_data = kzalloc((MAX_SZ_ZERO_BUF), GFP_KERNEL);
	if (!intelhaddata->flat_data) {
		retval = -ENOMEM;
		goto err;
	}

	intelhaddata->dev = &devptr->dev;
	pm_runtime_set_active(intelhaddata->dev);
	pm_runtime_enable(intelhaddata->dev);

	mutex_unlock(&had_mutex);
	retval = mid_hdmi_audio_register(&had_interface, intelhaddata);
	if (retval) {
		pr_err("registering with display driver failed %#x\n", retval);
		snd_card_free(card);
		goto free_hadstream;
	}

	return retval;
err:
	snd_card_free(card);
unlock_mutex:
	mutex_unlock(&had_mutex);
free_hadstream:
	kfree(had_stream);
	pm_runtime_disable(intelhaddata->dev);
	intelhaddata->dev = NULL;
free_haddata:
	kfree(intelhaddata);
	intelhaddata = NULL;
	pr_err("Error returned from %s api %#x\n", __func__, retval);
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
	struct snd_intelhad *intelhaddata = had_data;
	int caps;

	pr_debug("Enter %s\n", __func__);

	if (!intelhaddata)
		return 0;

	if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED) {
		caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		had_set_caps(HAD_SET_DISABLE_AUDIO_INT, &caps);
		had_set_caps(HAD_SET_DISABLE_AUDIO, NULL);
	}
	kfree(intelhaddata->flat_data);
	snd_card_free(intelhaddata->card);
	kfree(intelhaddata->private_data);
	kfree(intelhaddata);
	return 0;
}

static int had_pm_runtime_suspend(struct device *dev)
{
	return 0;
}

static int had_pm_runtime_resume(struct device *dev)
{
	return 0;
}

static int had_pm_runtime_idle(struct device *dev)
{
	return pm_schedule_suspend(dev, HAD_SUSPEND_DELAY);
}

const struct dev_pm_ops had_pm_ops = {
	.runtime_idle = had_pm_runtime_idle,
	.runtime_suspend = had_pm_runtime_suspend,
	.runtime_resume = had_pm_runtime_resume,
};

static struct platform_driver had_driver = {
	.probe =        hdmi_audio_probe,
	.remove		= __devexit_p(hdmi_audio_remove),
	.suspend =      NULL,
	.resume =       NULL,
	.driver		= {
		.name	= HDMI_AUDIO_DRIVER,
#ifdef CONFIG_PM
		.pm	= &had_pm_ops,
#endif
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
