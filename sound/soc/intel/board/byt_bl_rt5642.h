#ifndef _BYT_BL_RT5642_H_
#define _BYT_BL_RT5642_H_

#define MAX_CONTROLS	2

int byt_get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

int byt_set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

int byt_get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

int byt_set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

struct byt_comms_mc_private {
	bool ssp_bt_sco_master_mode;
	bool ssp_modem_master_mode;
};

/* Data path functionalities */
struct snd_pcm_hardware BYT_COMMS_BT_hw_param = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_DOUBLE |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_SYNC_START),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
	.rates = (SNDRV_PCM_RATE_8000),
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
	.buffer_bytes_max = (320*1024),
	.period_bytes_min = 32,
	.period_bytes_max = (320*1024),
	.periods_min = 2,
	.periods_max = (1024*2),
	.fifo_size = 0,
};

/* Data path functionalities */
struct snd_pcm_hardware BYT_COMMS_MODEM_hw_param = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_DOUBLE |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_SYNC_START),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
	.rates = (SNDRV_PCM_RATE_48000),
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = (640*1024),
	.period_bytes_min = 64,
	.period_bytes_max = (640*1024),
	.periods_min = 2,
	.periods_max = (1024*2),
	.fifo_size = 0,
};

/*
 * For Mixing 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define BYT_SSP_MIXING_SLOT_NB_SLOT	1
#define BYT_SSP_MIXING_SLOT_WIDTH	32
#define BYT_SSP_MIXING_SLOT_RX_MASK	0x1
#define BYT_SSP_MIXING_SLOT_TX_MASK	0x1

/*
 * For BT SCO 1 slot of 16 bits is used
 * to transfer mono 16 bits PCM samples
 */
#define BYT_SSP_BT_SLOT_NB_SLOT		1
#define BYT_SSP_BT_SLOT_WIDTH		16
#define BYT_SSP_BT_SLOT_RX_MASK		0x1
#define BYT_SSP_BT_SLOT_TX_MASK		0x1

/*
 * MIXER CONTROLS for SSP BT
 */
static const char *const ssp_master_mode_text[] = {"disabled", "enabled"};

static const struct soc_enum byt_ssp_bt_sco_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

static const struct soc_enum byt_ssp_modem_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

const struct snd_kcontrol_new byt_ssp_comms_controls[MAX_CONTROLS] = {
	SOC_ENUM_EXT("SSP BT Master Mode",
		 byt_ssp_bt_sco_master_mode_enum,
		 byt_get_ssp_bt_sco_master_mode,
		 byt_set_ssp_bt_sco_master_mode),
	SOC_ENUM_EXT("SSP Modem Master Mode",
		 byt_ssp_modem_master_mode_enum,
		 byt_get_ssp_modem_master_mode,
		 byt_set_ssp_modem_master_mode),
};

#endif /* _BYT_BL_RT5642_H_ */
