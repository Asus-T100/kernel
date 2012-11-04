/*
 *  sst_platform_pvt.h - Intel MID Platform driver header file
 *
 *  Copyright (C) 2010 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
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
 *
 *
 */

#ifndef __SST_PLATFORMDRVPVT_H__
#define __SST_PLATFORMDRVPVT_H__

/* TODO rmv this global */

extern struct sst_device *sst_dsp;
#define SST_MONO		1
#define SST_STEREO		2

#define SST_MIN_RATE		8000
#define SST_MAX_RATE		48000
#define SST_MIN_CHANNEL		1
#define SST_MAX_CHANNEL		2

#define SST_MAX_BUFFER		96000 /*500ms@48,16bit,2ch - CLV*/
#define SST_MIN_PERIOD_BYTES	1764  /*10ms@44.1,16bit,2ch - MFLD*/
#define SST_MAX_PERIOD_BYTES	48000 /*250ms@48,16bit,2ch - CLV*/

#define SST_MIN_PERIODS		2
#define SST_MAX_PERIODS		50
#define SST_FIFO_SIZE		0
#define SST_CLK_UNINIT		0x03
#define SST_CODEC_TYPE_PCM	1

#define SST_HEADSET_DAI "Headset-cpu-dai"
#define SST_SPEAKER_DAI "Speaker-cpu-dai"
#define SST_VIBRA1_DAI "Vibra1-cpu-dai"
#define SST_VIBRA2_DAI "Vibra2-cpu-dai"
#define SST_VOICE_DAI "Voice-cpu-dai"

struct sst_device;

enum sst_drv_status {
	SST_PLATFORM_UNINIT,
	SST_PLATFORM_INIT,
	SST_PLATFORM_RUNNING,
	SST_PLATFORM_PAUSED,
	SST_PLATFORM_DROPPED,
};


int __devinit sst_dsp_init(struct snd_soc_platform *platform);

unsigned int sst_soc_read(struct snd_soc_platform *platform,
		unsigned int reg);
int sst_soc_write(struct snd_soc_platform *platform,
		unsigned int reg, unsigned int val);

/* this section defines the map for the widgets
 * the widget layout is switches followed by mixers
 * each mixer will be represented by single value
 * and that value will have each input corrosping to a bit
 */
/* each out id will corrospond to one mixer and one path. Each input will be
 * represented by single bit in the register, same thing for big switch register
 * total out ids are 18, hence we use 32bit register for this
 */

/* output id's here,
 */
#define SST_MIX(x)		(x)

#define SST_MIX_MODEM		SST_MIX(0)
#define SST_MIX_BT		SST_MIX(1)
#define SST_MIX_CODEC0		SST_MIX(2)
#define SST_MIX_CODEC1		SST_MIX(3)
#define SST_MIX_LOOP0		SST_MIX(4)
#define SST_MIX_LOOP1		SST_MIX(5)
#define SST_MIX_LOOP2		SST_MIX(6)
#define SST_MIX_PROBE		SST_MIX(7)
#define SST_MIX_HF_SNS		SST_MIX(8)
#define SST_MIX_HF		SST_MIX(9)
#define SST_MIX_SPEECH		SST_MIX(10)
#define SST_MIX_RXSPEECH	SST_MIX(11)
#define SST_MIX_VOIP		SST_MIX(12)
#define SST_MIX_PCM0		SST_MIX(13)
#define SST_MIX_PCM1		SST_MIX(14)
#define SST_MIX_PCM2		SST_MIX(15)
#define SST_MIX_AWARE		SST_MIX(16)
#define SST_MIX_VAD		SST_MIX(17)
#define SST_MIX_MEDIA0		SST_MIX(18)
#define SST_MIX_MEDIA1		SST_MIX(19)
#define SST_MIX_VDL		SST_MIX(20)
#define SST_MIX_VUL		SST_MIX(21)
#define SST_MIX_VUL_R1		SST_MIX(22)
#define SST_MIX_VUL_R2		SST_MIX(23)
#define SST_MIX_AWARE_		SST_MIX(24)
#define SST_MIX_VAD_		SST_MIX(25)
#define SST_MIX_FM		SST_MIX(26)

#define SST_NUM_MIX		(SST_MIX_FM + 1)

#define SST_MIX_SWITCH		(SST_NUM_MIX + 1)
#define SST_OUT_SWITCH		(SST_NUM_MIX + 2)
#define SST_IN_SWITCH		(SST_NUM_MIX + 3)

/* last entry defines array size */
#define SST_NUM_WIDGETS		(SST_IN_SWITCH + 1)

/* input ids */
/* in each mixer we will define bitfield for each inputs to be taken
 */
#define SST_MIX_IP(x)		(x)

#define SST_IP_MODEM		SST_MIX_IP(0)
#define SST_IP_BT		SST_MIX_IP(1)
#define SST_IP_CODEC0		SST_MIX_IP(2)
#define SST_IP_CODEC1		SST_MIX_IP(3)
#define SST_IP_LOOP0		SST_MIX_IP(4)
#define SST_IP_LOOP1		SST_MIX_IP(5)
#define SST_IP_LOOP2		SST_MIX_IP(6)
#define SST_IP_PROBE		SST_MIX_IP(7)
#define SST_IP_SIDETONE		SST_MIX_IP(8)
#define SST_IP_TxSPEECH		SST_MIX_IP(9)
#define SST_IP_SPEECH		SST_MIX_IP(10)
#define SST_IP_TONE		SST_MIX_IP(11)
#define SST_IP_VOIP		SST_MIX_IP(12)
#define SST_IP_PCM0		SST_MIX_IP(13)
#define SST_IP_PCM1		SST_MIX_IP(14)
#define SST_IP_MEDIA0		SST_MIX_IP(15)
#define SST_IP_MEDIA1		SST_MIX_IP(16)
#define SST_IP_MEDIA2		SST_MIX_IP(17)
#define SST_IP_FM		SST_MIX_IP(18)


struct sst_data {
	struct platform_device *pdev;
	struct sst_platform_data *pdata;
	unsigned int	lpe_mixer_input_ihf;
	unsigned int	lpe_mixer_input_hs;
	u16 widget[SST_NUM_WIDGETS];
	char *byte_stream;
	struct mutex lock;
};
#endif
