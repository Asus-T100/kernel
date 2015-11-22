/*
 *  probe_point_dpcm.c - Intel MID probe definition
 *
 *  Copyright (C) 2014 Intel Corp
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

static const struct sst_probe_config sst_probes[] = {
	/* TODO: get this struct from FW config data */
	/* gain outputs  */
	{ "pcm0_in gain", SST_PATH_INDEX_PCM0_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm1_in gain", SST_PATH_INDEX_PCM1_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm1_out gain", SST_PATH_INDEX_PCM1_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm2_out gain", SST_PATH_INDEX_PCM2_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm3_out gain", SST_PATH_INDEX_PCM3_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm4_out gain", SST_PATH_INDEX_PCM4_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "voip_in gain", SST_PATH_INDEX_VOIP_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "voip_out gain", SST_PATH_INDEX_VOIP_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "aware_out gain", SST_PATH_INDEX_AWARE_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "vad_out gain", SST_PATH_INDEX_VAD_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_out gain", SST_PATH_INDEX_HF_SNS_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_3_out gain", SST_PATH_INDEX_HF_SNS_3_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_4_out gain", SST_PATH_INDEX_HF_SNS_4_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_out gain", SST_PATH_INDEX_HF_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "speech_out gain", SST_PATH_INDEX_SPEECH_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "txspeech_in gain", SST_PATH_INDEX_TX_SPEECH_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "rxspeech_out gain", SST_PATH_INDEX_RX_SPEECH_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "speech_in gain", SST_PATH_INDEX_SPEECH_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop1_out gain", SST_PATH_INDEX_MEDIA_LOOP1_OUT , SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop2_out gain", SST_PATH_INDEX_MEDIA_LOOP2_OUT , SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "tone_in gain", SST_PATH_INDEX_TONE_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_out0 gain", SST_PATH_INDEX_CODEC_OUT0, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_out1 gain", SST_PATH_INDEX_CODEC_OUT1, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_out gain", SST_PATH_INDEX_BT_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_out gain", SST_PATH_INDEX_FM_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "modem_out gain", SST_PATH_INDEX_MODEM_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in0 gain", SST_PATH_INDEX_CODEC_IN0, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in1 gain", SST_PATH_INDEX_CODEC_IN1, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_in gain", SST_PATH_INDEX_BT_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_in gain", SST_PATH_INDEX_FM_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "modem_in gain", SST_PATH_INDEX_MODEM_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "sprot_loop_out gain", SST_PATH_INDEX_SPROT_LOOP_OUT, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },
	{ "sidetone_in", SST_PATH_INDEX_SIDETONE_IN, SST_MODULE_ID_GAIN_CELL, SST_TASK_SBA, { 1, 2, 1 } },

	/* SRC */
	{ "media0_in src", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_SRC, SST_TASK_MMX, { 1, 2, 1 } },
	{ "rxspeech_out src", SST_PATH_INDEX_RX_SPEECH_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "txspeech_in src", SST_PATH_INDEX_TX_SPEECH_IN, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "speech_out src", SST_PATH_INDEX_SPEECH_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "speech_in src", SST_PATH_INDEX_SPEECH_IN, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_out src", SST_PATH_INDEX_HF_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_out src", SST_PATH_INDEX_HF_SNS_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_3_out src", SST_PATH_INDEX_HF_SNS_3_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_4_out src", SST_PATH_INDEX_HF_SNS_4_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm1_out src", SST_PATH_INDEX_PCM1_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "pcm2_out src", SST_PATH_INDEX_PCM2_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "voip_in src", SST_PATH_INDEX_VOIP_IN, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "voip_out src", SST_PATH_INDEX_VOIP_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "aware_out src", SST_PATH_INDEX_AWARE_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "vad_out src", SST_PATH_INDEX_VAD_OUT, SST_MODULE_ID_SRC, SST_TASK_SBA, { 1, 2, 1 } },

	{ "media0_in downmix", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_DOWNMIX, SST_TASK_MMX, { 1, 2, 1 } },
	{ "sprot_loop_out lpro", SST_PATH_INDEX_SPROT_LOOP_OUT, SST_MODULE_ID_SPROT, SST_TASK_SBA, { 1, 2, 1 } },

	{ "voice_downlink 9", FBA_PROBE_INDEX_9, 0, SST_TASK_FBA_DL , { 0, 0, 0 } },
	{ "voice_downlink 10", FBA_PROBE_INDEX_10, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 11", FBA_PROBE_INDEX_11, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 12", FBA_PROBE_INDEX_12, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 13", FBA_PROBE_INDEX_13, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 14", FBA_PROBE_INDEX_14, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 17", FBA_PROBE_INDEX_17, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 18", FBA_PROBE_INDEX_18, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 21", FBA_PROBE_INDEX_21, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_downlink 27", FBA_PROBE_INDEX_27, 0, SST_TASK_FBA_DL, { 0, 0, 0 } },
	{ "voice_uplink 1", FBA_PROBE_INDEX_1, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 2", FBA_PROBE_INDEX_2, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 3", FBA_PROBE_INDEX_3, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 4", FBA_PROBE_INDEX_4, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 5", FBA_PROBE_INDEX_5, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 6", FBA_PROBE_INDEX_6, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 7", FBA_PROBE_INDEX_7, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 8", FBA_PROBE_INDEX_8, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 15", FBA_PROBE_INDEX_15, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 16", FBA_PROBE_INDEX_16, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 19", FBA_PROBE_INDEX_19, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 20", FBA_PROBE_INDEX_20, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 22", FBA_PROBE_INDEX_22, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 23", FBA_PROBE_INDEX_23, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 24", FBA_PROBE_INDEX_24, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 25", FBA_PROBE_INDEX_25, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },
	{ "voice_uplink 26", FBA_PROBE_INDEX_26, 0, SST_TASK_FBA_UL, { 0, 0, 0 } },


	{ "media_loop1_out mdrp", SST_PATH_INDEX_MEDIA_LOOP1_OUT , SST_MODULE_ID_MDRP, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop2_out mdrp", SST_PATH_INDEX_MEDIA_LOOP2_OUT , SST_MODULE_ID_MDRP, SST_TASK_SBA, { 1, 2, 1 } },

	{ "media_loop1_out fir_stereo", SST_PATH_INDEX_MEDIA_LOOP1_OUT , SST_MODULE_ID_FIR_24, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop2_out fir_stereo", SST_PATH_INDEX_MEDIA_LOOP2_OUT , SST_MODULE_ID_FIR_24, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop1_out iir_stereo", SST_PATH_INDEX_MEDIA_LOOP1_OUT , SST_MODULE_ID_IIR_24, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media_loop2_out iir_stereo", SST_PATH_INDEX_MEDIA_LOOP2_OUT , SST_MODULE_ID_IIR_24, SST_TASK_SBA, { 1, 2, 1 } },
	{ "sidetone_in iir_stereo", SST_PATH_INDEX_SIDETONE_IN, SST_MODULE_ID_IIR_24, SST_TASK_SBA, { 1, 2, 1 } },
	{ "media2_out input", SST_PATH_INDEX_MEDIA2_OUT , 0xFF00, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media2_out bmf", SST_PATH_INDEX_MEDIA2_OUT , SST_MODULE_ID_MEDIA_BMF, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media2_out wnr", SST_PATH_INDEX_MEDIA2_OUT , SST_MODULE_ID_MEDIA_WNR, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media2_out agc", SST_PATH_INDEX_MEDIA2_OUT , SST_MODULE_ID_MEDIA_AGC, SST_TASK_MMX, { 1, 2, 1 } },

	/* ASRC */
	{ "modem_out asrc", SST_PATH_INDEX_MODEM_OUT, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "modem_in asrc", SST_PATH_INDEX_MODEM_IN, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_out asrc", SST_PATH_INDEX_BT_OUT, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_in asrc", SST_PATH_INDEX_BT_IN, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_out asrc", SST_PATH_INDEX_FM_OUT, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_in asrc", SST_PATH_INDEX_FM_IN, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_out0 asrc", SST_PATH_INDEX_CODEC_OUT0, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_out1 asrc", SST_PATH_INDEX_CODEC_OUT1, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in0 asrc", SST_PATH_INDEX_CODEC_IN0, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in1 asrc", SST_PATH_INDEX_CODEC_IN1, SST_MODULE_ID_ASRC, SST_TASK_SBA, { 1, 2, 1 } },

	{ "tone_in tone_gen", SST_PATH_INDEX_TONE_IN, SST_MODULE_ID_TONE_GEN, SST_TASK_SBA, { 1, 2, 1 } },
	{ "hf_sns_out bmf", SST_PATH_INDEX_HF_SNS_OUT, SST_MODULE_ID_BMF, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_3_out bmf", SST_PATH_INDEX_HF_SNS_3_OUT, SST_MODULE_ID_BMF, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_4_out bmf", SST_PATH_INDEX_HF_SNS_4_OUT, SST_MODULE_ID_BMF, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_out edl", SST_PATH_INDEX_HF_OUT, SST_MODULE_ID_EDL, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_out fir", SST_PATH_INDEX_HF_SNS_OUT, SST_MODULE_ID_FIR_16, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_out iir", SST_PATH_INDEX_HF_SNS_OUT, SST_MODULE_ID_IIR_16, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_3_out fir", SST_PATH_INDEX_HF_SNS_3_OUT, SST_MODULE_ID_FIR_16, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_3_out iir", SST_PATH_INDEX_HF_SNS_3_OUT, SST_MODULE_ID_IIR_16, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_4_out fir", SST_PATH_INDEX_HF_SNS_4_OUT, SST_MODULE_ID_FIR_16, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_4_out iir", SST_PATH_INDEX_HF_SNS_4_OUT, SST_MODULE_ID_IIR_16, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_out ref_line", SST_PATH_INDEX_HF_OUT, SST_MODULE_ID_REF_LINE, SST_TASK_FBA_UL, { 1, 2, 1 } },

	{ "media0_in volume", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_VOLUME, SST_TASK_MMX, { 1, 2, 1 } },

	/* DCR */
	{ "modem_in dcr", SST_PATH_INDEX_MODEM_IN, SST_MODULE_ID_FILT_DCR, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_in dcr", SST_PATH_INDEX_BT_IN, SST_MODULE_ID_FILT_DCR, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_in dcr", SST_PATH_INDEX_FM_IN, SST_MODULE_ID_FILT_DCR, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in0 dcr", SST_PATH_INDEX_CODEC_IN0, SST_MODULE_ID_FILT_DCR, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in1 dcr", SST_PATH_INDEX_CODEC_IN1, SST_MODULE_ID_FILT_DCR, SST_TASK_SBA, { 1, 2, 1 } },

	/* Log */
	{ "modem_out log", SST_PATH_INDEX_MODEM_OUT, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "modem_in log", SST_PATH_INDEX_MODEM_IN, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_out log", SST_PATH_INDEX_BT_OUT, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "bt_in log", SST_PATH_INDEX_BT_IN, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_in log", SST_PATH_INDEX_FM_IN, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "fm_out log", SST_PATH_INDEX_FM_OUT, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_out0 log", SST_PATH_INDEX_CODEC_OUT0, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "codec_in0 log", SST_PATH_INDEX_CODEC_IN0, SST_MODULE_ID_LOG, SST_TASK_SBA, { 1, 2, 1 } },
	{ "voice_downlink log", SST_PATH_INDEX_VOICE_DOWNLINK, SST_MODULE_ID_LOG, SST_TASK_FBA_DL, { 1, 2, 1 } },
	{ "voice_uplink log", SST_PATH_INDEX_VOICE_UPLINK, SST_MODULE_ID_LOG, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_out log", SST_PATH_INDEX_HF_OUT, SST_MODULE_ID_LOG, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_out log", SST_PATH_INDEX_HF_SNS_OUT, SST_MODULE_ID_LOG, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_3_out log", SST_PATH_INDEX_HF_SNS_3_OUT, SST_MODULE_ID_LOG, SST_TASK_FBA_UL, { 1, 2, 1 } },
	{ "hf_sns_4_out log", SST_PATH_INDEX_HF_SNS_4_OUT, SST_MODULE_ID_LOG, SST_TASK_FBA_UL, { 1, 2, 1 } },

	/* Decoder */
	{ "media0_in pcm", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_PCM, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in mp3", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_MP3, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in mp24", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_MP24, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in aac", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_AAC, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in aacp", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_AACP, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in eaacp", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_EAACP, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in wma9", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_WMA9, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in wma10", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_WMA10, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in wma10p", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_WMA10P, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in ra", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_RA, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in ddac3", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_DDAC3, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in true_hd", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_TRUE_HD, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in hd_plus", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_HD_PLUS, SST_TASK_MMX, { 1, 2, 1 } },

	/* Effects */
	{ "media0_in bass_boost", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_BASS_BOOST, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in stereo_wdng", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_STEREO_WDNG, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in av_removal", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_AV_REMOVAL, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in mic_eq", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_MIC_EQ, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in spl", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_SPL, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in vtsv", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_ALGO_VTSV, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in virtualizer", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_VIRTUALIZER, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in visualization", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_VISUALIZATION, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in loudness_optimizer", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_LOUDNESS_OPTIMIZER, SST_TASK_MMX, { 1, 2, 1 } },
	{ "media0_in reverberation", SST_PATH_INDEX_MEDIA0_IN, SST_MODULE_ID_REVERBERATION, SST_TASK_MMX, { 1, 2, 1 } },
};

