/*
 *  mfld_ssp_wl1273_machine.c - ASoc Machine driver for
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Selma Bensaid <selma.bensaidl@intel.com>
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
 */

#define FORMAT(fmt) "%s: " fmt, __func__
#define pr_fmt(fmt) KBUILD_MODNAME ": " FORMAT(fmt)

#include "mfld_comms_machine.h"

/*****************************************/
/* Global Variables                      */
/*****************************************/

static struct snd_soc_ops mfld_comms_dai_link_ops = {
		.startup = mfld_comms_dai_link_startup,
		.hw_params = mfld_comms_dai_link_hw_params,
};

/*
 * MIXER CONTROLS for BT SCO stream
 */
static const char * const ssp_master_mode_text[] = {"disabled", "enabled"};
static const struct soc_enum ssp_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);


static int get_ssp_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct comms_mc_private *comms_ctx =
				snd_soc_card_get_drvdata(card);
	ucontrol->value.integer.value[0] = comms_ctx->ssp_master_mode;
	return 0;
}

static int set_ssp_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct comms_mc_private *comms_ctx =
				snd_soc_card_get_drvdata(card);

	if (ucontrol->value.integer.value[0] == comms_ctx->ssp_master_mode)
		return 0;

	comms_ctx->ssp_master_mode = ucontrol->value.integer.value[0];

	return 0;
}


static const struct snd_kcontrol_new ssp_snd_controls[] = {
	SOC_ENUM_EXT("SSP Master Mode", ssp_master_mode_enum,
			get_ssp_master_mode, set_ssp_master_mode),
};



static int mfld_comms_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_card *card = runtime->card;
	struct comms_mc_private *comms_ctx =
					snd_soc_card_get_drvdata(card);
	int ret_val;

	comms_ctx->ssp_master_mode = false;

	ret_val = snd_soc_add_card_controls(card, ssp_snd_controls,
				ARRAY_SIZE(ssp_snd_controls));
	if (ret_val)
		pr_err("MFLD Comms Machine: Init failed %d",
				ret_val);


	return ret_val;
}

/*
 * DAI LINK DEFINITIONS
 */


struct snd_soc_dai_link mfld_comms_dailink[] = {
	[BT_SCO_DEV] = {
		.name = "BT SCO",
		.stream_name = "BTSCO",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "wl1273_BT",
		.codec_name = "wl1273_uart",
		.platform_name = "mid-ssp-dai",
		.init = mfld_comms_init,
		.ops = &mfld_comms_dai_link_ops,
	},
	[FM_DEV] = {
		.name = "FM",
		.stream_name = "FM",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "wl1273_FM",
		.codec_name = "wl1273_uart",
		.platform_name = "mid-ssp-dai",
		.init = mfld_comms_init,
		.ops = &mfld_comms_dai_link_ops,
	},
	[MSIC_VOIP_DEV] = {
		.name = "MSIC VOIP",
		.stream_name = "VOIP",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "es305 Voice",
		.codec_name = "es305_codec",
		.platform_name = "mid-ssp-dai",
		.init = mfld_comms_init,
		.ops = &mfld_comms_dai_link_ops,
	},
	[IFX_MODEM_DEV] = {
		.name = "IFX MODEM",
		.stream_name = "IFX_MODEM_MIXING",
		.cpu_dai_name = SSP_MODEM_DAI_NAME,
		.codec_dai_name = "IFX_Modem_Mixing",
		.codec_name = "ifx_modem",
		.platform_name = "mid-ssp-dai",
		.init = mfld_comms_init,
		.ops = &mfld_comms_dai_link_ops,
	},
};

/*
 * DAI LINK OPS
 */

static int mfld_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "MFLD Comms Machine: ERROR "
			"NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case BT_SCO_DEV:
		str_runtime->hw = BT_soc_hw_param;
		break;
	case FM_DEV:
		str_runtime->hw = FM_soc_hw_param;
		break;
	case MSIC_VOIP_DEV:
		str_runtime->hw = VOIP_alsa_hw_param;
		break;
	case IFX_MODEM_DEV:
		str_runtime->hw = IFX_modem_alsa_hw_param;
		break;
	default:
		pr_err("MFLD Comms Machine: bad PCM Device = %d\n",
						substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
}


static int mfld_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct comms_mc_private *comms_ctx =
					snd_soc_card_get_drvdata(soc_card);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
	unsigned int device = substream->pcm->device;


	switch (device) {
	case BT_SCO_DEV:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_1
		 * sspslclk_direction = SSPSCLK_MASTER_MODE
		 * sspsfrm_direction = SSPSFRM_MASTER_MODE
		 */

		ret = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_IF |
					(comms_ctx->ssp_master_mode ?
					SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("MFLD Comms Machine: Set FMT Fails %d\n", ret);
			return -EINVAL;
		}

		/*
		 * BT SCO SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 16
		 * tristate = 1
		 * ssp_frmsync_timing_bit = 0
		 * (NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 * ssp_frmsync_timing_bit = 1
		 * (NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * ssp_psp_T2 = 1
		 * (Dummy start offset = 1 bit clock period)
		 */
		nb_slot = SSP_BT_SLOT_NB_SLOT;
		slot_width = SSP_BT_SLOT_WIDTH;
		tx_mask = SSP_BT_SLOT_TX_MASK;
		rx_mask = SSP_BT_SLOT_RX_MASK;

		if (comms_ctx->ssp_master_mode)
			tristate_offset = BIT(TRISTATE_BIT) |
			BIT(DUMMY_START_ONE_PERIOD_OFFSET);
		else
			tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		break;
	case FM_DEV:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_1
		 * sspslclk_direction = SSPSCLK_MASTER_MODE
		 * sspsfrm_direction = SSPSFRM_MASTER_MODE
		 */

		ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
				(comms_ctx->ssp_master_mode ?
				SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("MFLD Comms Machine: Set FMT Fails %d\n", ret);
			return -EINVAL;
		}

		/*
		 * FM SSP Config
		 * ssp_active_tx_slots_map = 0x00
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * tristate = 1
		 * ssp_frmsync_timing_bit = 1
		 * (NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 *
		 */
		nb_slot = SSP_FM_SLOT_NB_SLOT;
		slot_width = SSP_FM_SLOT_WIDTH;
		tx_mask = SSP_FM_SLOT_TX_MASK;
		rx_mask = SSP_FM_SLOT_RX_MASK;
		tristate_offset = BIT(TRISTATE_BIT) |
					BIT(DUMMY_START_ONE_PERIOD_OFFSET);
		break;
	case MSIC_VOIP_DEV:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_1
		 * sspslclk_direction = SSPSCLK_SLAVE_MODE
		 * sspsfrm_direction = SSPSFRM_SLAVE_MODE
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
				(comms_ctx->ssp_master_mode ?
				SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("MFLD Comms Machine: Set FMT Fails %d\n",
							ret);
			return -EINVAL;
		}

		/*
		 * MSIC VOIP SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * tristate = 1
		 * ssp_frmsync_timing_bit = 0, for SLAVE
		 * (NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 * ssp_frmsync_timing_bit = 1, for MASTER
		 * (NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 *
		 *
		 */
		nb_slot = SSP_VOIP_SLOT_NB_SLOT;
		slot_width = SSP_VOIP_SLOT_WIDTH;
		tx_mask = SSP_VOIP_SLOT_TX_MASK;
		rx_mask = SSP_VOIP_SLOT_RX_MASK;

		if (comms_ctx->ssp_master_mode)
			tristate_offset = BIT(TRISTATE_BIT) |
					BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		else
			tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		break;
	case IFX_MODEM_DEV:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_1
		 * Master:
		 *	sspslclk_direction = SSPSCLK_MASTER_MODE
		 *	sspsfrm_direction = SSPSFRM_MASTER_MODE
		 * Slave:
		 *	sspslclk_direction = SSPSCLK_SLAVE_MODE
		 *	sspsfrm_direction = SSPSFRM_SLAVE_MODE
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
				(comms_ctx->ssp_master_mode ?
				SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));
		if (ret < 0) {
			pr_err("MFLD Comms Machine:  Set FMT Fails %d\n", ret);
			return -EINVAL;
		}

		/*
		 * IFX Modem Mixing SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * Master:
		 *	tristate = 3
		 *	ssp_frmsync_timing_bit = 1, for MASTER
		 *	(NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * Slave:
		 *	tristate = 1
		 *	ssp_frmsync_timing_bit = 0, for SLAVE
		 *	(NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 *
		 */
		nb_slot = SSP_IFX_SLOT_NB_SLOT;
		slot_width = SSP_IFX_SLOT_WIDTH;
		tx_mask = SSP_IFX_SLOT_TX_MASK;
		rx_mask = SSP_IFX_SLOT_RX_MASK;

		if (comms_ctx->ssp_master_mode)
			tristate_offset = BIT(TRISTATE_BIT) |
					BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		else
			tristate_offset = BIT(TRISTATE_BIT);
		break;
	default:
		pr_err("MFLD Comms Machine: bad PCM Device ID = %d\n", device);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
			rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("MFLD Comms Machine:  Set TDM Slot Fails %d\n", ret);
		return -EINVAL;
	}

	/* select clock source (if master) */
	/* BT SCO: CPU DAI is master */
	/* FM: CPU DAI is master */
	/* BT_VOIP: CPU DAI is master */
	if ((device == BT_SCO_DEV && comms_ctx->ssp_master_mode) ||
		(device == FM_DEV) ||
		((device == MSIC_VOIP_DEV) && comms_ctx->ssp_master_mode) ||
		((device == IFX_MODEM_DEV) && comms_ctx->ssp_master_mode))  {

		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->hw.rates, 0);
	}


	pr_info("MFLD Comms Machine: slot_width = %d\n",
			slot_width);
	pr_info("MFLD Comms Machine: tx_mask = %d\n",
			tx_mask);
	pr_info("MFLD Comms Machine: rx_mask = %d\n",
			rx_mask);
	pr_info("MFLD Comms Machine: tristate_offset = %d\n",
			tristate_offset);

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("MFLD Comms Machine: Set Tristate Fails %d\n",
						ret);
		return -EINVAL;
	}


	return 0;
} /* mfld_comms_dai_link_hw_params*/

/* SoC card */
static struct snd_soc_card snd_comms_card_mfld = {
	.name = "SspCommsAudio",
	.dai_link = mfld_comms_dailink,
	.num_links = ARRAY_SIZE(mfld_comms_dailink),
};

static int __devinit snd_mfld_comms_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct comms_mc_private *comms_ctx;

	pr_info("MFLD Comms Machine: FCT %s enters\n",
			__func__);

	/* register the soc card */
	snd_comms_card_mfld.dev = &pdev->dev;

	snd_soc_initialize_card_lists(&snd_comms_card_mfld);

	ret_val = snd_soc_register_card(&snd_comms_card_mfld);
	if (ret_val) {
		pr_err("MFLD Comms Machine: card "
				"registeration failed %d\n",
				ret_val);
		return -EBUSY;
	}

	comms_ctx = kzalloc(sizeof(struct comms_mc_private), GFP_KERNEL);
	if (comms_ctx == NULL) {
		pr_err("Unable to allocate comms_ctx\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, &snd_comms_card_mfld);
	snd_soc_card_set_drvdata(&snd_comms_card_mfld, comms_ctx);

	return 0;
} /* snd_mfld_comms_probe */

static int __devexit snd_mfld_comms_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct comms_mc_private *comms_ctx = snd_soc_card_get_drvdata(soc_card);

	pr_info("MFLD Comms Machine: FCT %s enters\n",
			__func__);
	kfree(comms_ctx);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(&snd_comms_card_mfld);
	platform_set_drvdata(pdev, NULL);
	return 0;
} /* snd_mfld_comms_remove */


static struct platform_driver snd_mfld_comms_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mfld-comms-audio",
	},
	.probe = snd_mfld_comms_probe,
	.remove = __devexit_p(snd_mfld_comms_remove),
};

static int __init snd_mfld_comms_init(void)
{
	pr_info("MFLD Comms Machine: FCT %s enters\n",
			__func__);
	return platform_driver_register(&snd_mfld_comms_driver);
}
module_init(snd_mfld_comms_init);

static void __exit snd_mfld_comms_exit(void)
{
	pr_info("MFLD Comms Machine: FCT %s enters\n",
			__func__);
	platform_driver_unregister(&snd_mfld_comms_driver);
}
module_exit(snd_mfld_comms_exit);

MODULE_DESCRIPTION("ASoC Intel(R) MID Machine driver");
MODULE_AUTHOR("Selma Bensaid");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ssp-bt-fm-audio");
