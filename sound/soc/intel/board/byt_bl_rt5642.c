/*
 *  byt_bl_rt5642.c - ASoc Machine driver for Intel Baytrail Baylake MID platform
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/vlv2_plat_clock.h>
#include <linux/acpi_gpio.h>
#include <linux/extcon-mid.h>
#include <asm/platform_byt_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/rt5640.h"
#include "byt_bl_rt5642.h" //<asus-baron20131101+>
#include "../ssp/mid_ssp.h" //<asus-baron20131101+>

#define BYT_PLAT_CLK_3_HZ	25000000

static int debounce = 100;
module_param(debounce, int, 0644);


struct byt_mc_private {
	struct byt_comms_mc_private comms_ctl; //<asus-baron20131101+>
	struct snd_soc_jack jack;
};
//<asus-baron20131101+>
int byt_get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;
	return 0;
}

int byt_set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_bt_sco_master_mode)
		return 0;

	ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];

	return 0;
}

int byt_get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_modem_master_mode;
	return 0;
}

int byt_set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_modem_master_mode)
		return 0;

	ctl->ssp_modem_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
//<asus-baron20131101->

static int byt_hp_detection(void);
static struct snd_soc_jack_gpio hs_gpio = {
		.name			= "byt-codec-int",
		.report			= SND_JACK_HEADSET |
					  SND_JACK_HEADPHONE |
					  SND_JACK_BTN_0,
//<asus-baron20130923->		.debounce_time		= 100,
		.debounce_time		= 500, //<asus-baron20130923+>
		.jack_status_check	= byt_hp_detection,
};

static inline void byt_jack_report(int status)
{
	switch (status) {
	case SND_JACK_HEADPHONE:
		mid_extcon_headset_report(HEADSET_NO_MIC);
		break;
	case SND_JACK_HEADSET:
		mid_extcon_headset_report(HEADSET_WITH_MIC);
		break;
	default:
		mid_extcon_headset_report(HEADSET_PULL_OUT);
		break;
	}
	pr_debug("%s: headset reported: 0x%x\n", __func__, status);
}

static void set_mic_bias(struct snd_soc_codec *codec,
			 const char *bias_widget, bool enable)
{
	pr_debug("%s %s\n", enable ? "enable" : "disable", bias_widget);
	if (enable)
		snd_soc_dapm_force_enable_pin(&codec->dapm, bias_widget);
	else
		snd_soc_dapm_disable_pin(&codec->dapm, bias_widget);
	snd_soc_dapm_sync(&codec->dapm);
}

static int byt_hp_detection(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	int status, jack_type = 0;

	pr_debug("Enter:%s", __func__);
	status = rt5640_check_interrupt_event(codec);
	switch (status) {
	case RT5640_J_IN_EVENT:
		pr_debug("Jack insert intr");
		set_mic_bias(codec, "micbias1", true);
		set_mic_bias(codec, "LDO2", true);
		status = rt5640_headset_detect(codec, true);
		if (status == RT5640_HEADPHO_DET) {
			jack_type = SND_JACK_HEADPHONE;
			pr_err("<asus-baron> SND_JACK_HEADPHONE\n"); //<asus-baron20130923+>
		}
		else if (status == RT5640_HEADSET_DET) {
			jack_type = SND_JACK_HEADSET;
			pr_err("<asus-baron> SND_JACK_HEADSET\n"); //<asus-baron20130923+>
			if (debounce)
				gpio->debounce_time = debounce;
			pr_debug("debounce = %d\n", gpio->debounce_time);
		} else /* RT5640_NO_JACK */
			jack_type = 0;

		byt_jack_report(jack_type);

		if (jack_type != SND_JACK_HEADSET) {
			set_mic_bias(codec, "micbias1", false);
			set_mic_bias(codec, "LDO2", false);
		}

		pr_debug("Jack type detected:%d", jack_type);
		break;
	case RT5640_J_OUT_EVENT:
		pr_debug("Jack remove intr");
		pr_err("<asus-baron> Jack remove intr\n"); //<asus-baron20130923+>
//<asus-baron20130923->		gpio->debounce_time = 100;
		gpio->debounce_time = 500; //<asus-baron20130923+>
		status = rt5640_headset_detect(codec, false);
		jack_type = 0;
		byt_jack_report(jack_type);
		set_mic_bias(codec, "micbias1", false);
		set_mic_bias(codec, "LDO2", false);
		break;
	case RT5640_BR_EVENT:
		pr_debug("BR event received");
		jack_type = SND_JACK_HEADSET;
		break;
	case RT5640_BP_EVENT:
		pr_debug("BP event received");
		jack_type = SND_JACK_HEADSET | SND_JACK_BTN_0;
		break;
	case RT5640_UN_EVENT:
		pr_debug("Reported invalid/RT5640_UN_EVENT");
		/* return previous status */
		jack_type = jack->status;
		break;
	default:
		pr_err("Error: Invalid event");
	}
	return jack_type;
}

static inline struct snd_soc_codec *byt_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "rt5640.2-001c")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2
static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	codec = byt_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; Unable to set platform clock\n");
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_ON);
		pr_debug("Platform clk turned ON\n");
		snd_soc_codec_set_sysclk(codec, RT5640_SCLK_S_PLL1,
				0, BYT_PLAT_CLK_3_HZ, SND_SOC_CLOCK_IN);
	} else {
		/* Set codec clock source to internal clock before
		   turning off the platform clock. Codec needs clock
		   for Jack detection and button press */
		snd_soc_codec_set_sysclk(codec, RT5640_SCLK_S_RCCLK,
				0, 0, SND_SOC_CLOCK_IN);
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_OFF);
		pr_debug("Platform clk turned OFF\n");
	}

	return 0;
}

static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route byt_audio_map[] = {
	{"IN2P", NULL, "Headset Mic"},
	{"IN2N", NULL, "Headset Mic"},
	//{"DMIC1", NULL, "Int Mic"}, //<asus-baron20130823->
	{"micbias1", NULL, "Int Mic"}, //<asus-baron20130823+>
	{"IN1P", NULL, "micbias1"}, //<asus-baron20130823+>
	{"IN1N", NULL, "micbias1"},	//<asus-baron20130823+>
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk", NULL, "SPOLP"},
	{"Ext Spk", NULL, "SPOLN"},
	{"Ext Spk", NULL, "SPORP"},
	{"Ext Spk", NULL, "SPORN"},

	{"Headphone", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
};

static const struct snd_kcontrol_new byt_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};


static int byt_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	pr_debug("Enter:%s", __func__);
	/* I2S Slave Mode`*/
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	      SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5640_PLL1_S_MCLK,
				  BYT_PLAT_CLK_3_HZ, params_rate(params) * 512);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}
	return 0;
}

static int byt_aif2_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	pr_debug("Enter:%s", __func__);
	/* I2S  Slave Mode`*/
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	      SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5640_PLL1_S_MCLK,
				  BYT_PLAT_CLK_3_HZ, params_rate(params) * 512);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}
	return 0;
}

static int byt_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
	}
	card->dapm.bias_level = level;
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);
	return 0;
}
//<asus-baron20131101+>
static int byt_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "BYT Comms Machine: ERROR NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

    /* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case BYT_COMMS_BT:
	str_runtime->hw = BYT_COMMS_BT_hw_param;
	break;

	case BYT_COMMS_MODEM:
	str_runtime->hw = BYT_COMMS_MODEM_hw_param;
	break;
	default:
	pr_err("BYT Comms Machine: bad PCM Device = %d\n",
	       substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
					 SNDRV_PCM_HW_PARAM_PERIODS);
}

static int byt_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
	unsigned int device = substream->pcm->device;


	pr_debug("ssp_bt_sco_master_mode %d\n", ctl->ssp_bt_sco_master_mode);
	pr_debug("ssp_modem_master_mode %d\n", ctl->ssp_modem_master_mode);

	switch (device) {
	case BYT_COMMS_BT:
	/*
	 * set cpu DAI configuration
	 * frame_format = PSP_FORMAT
	 * ssp_serial_clk_mode = SSP_CLK_MODE_1
	 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
	 */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SSP_DAI_SCMODE_1 |
				  SND_SOC_DAIFMT_NB_NF |
				  (ctl->ssp_bt_sco_master_mode ?
				   SND_SOC_DAIFMT_CBM_CFM :
				   SND_SOC_DAIFMT_CBS_CFS));

	if (ret < 0) {
		pr_err("BYT Comms Machine: Set FMT Fails %d\n",
			ret);
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
	nb_slot = BYT_SSP_BT_SLOT_NB_SLOT;
	slot_width = BYT_SSP_BT_SLOT_WIDTH;
	tx_mask = BYT_SSP_BT_SLOT_TX_MASK;
	rx_mask = BYT_SSP_BT_SLOT_RX_MASK;

	if (ctl->ssp_bt_sco_master_mode)
		tristate_offset = BIT(TRISTATE_BIT);
	else
		tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
	break;

	case BYT_COMMS_MODEM:
	/*
	 * set cpu DAI configuration
	 * frame_format = PSP_FORMAT
	 * ssp_serial_clk_mode = SSP_CLK_MODE_0
	 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
	 */
	ret = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_I2S |
					SSP_DAI_SCMODE_0 |
					SND_SOC_DAIFMT_NB_NF |
					(ctl->ssp_modem_master_mode ?
					SND_SOC_DAIFMT_CBM_CFM :
					SND_SOC_DAIFMT_CBS_CFS));
	if (ret < 0) {
		pr_err("BYT Comms Machine:  Set FMT Fails %d\n", ret);
		return -EINVAL;
	}

	/*
	 * Modem Mixing SSP Config
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
	nb_slot = BYT_SSP_MIXING_SLOT_NB_SLOT;
	slot_width = BYT_SSP_MIXING_SLOT_WIDTH;
	tx_mask = BYT_SSP_MIXING_SLOT_TX_MASK;
	rx_mask = BYT_SSP_MIXING_SLOT_RX_MASK;

	tristate_offset = BIT(TRISTATE_BIT) |\
	    BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);

	break;
	default:
	pr_err("BYT Comms Machine: bad PCM Device ID = %d\n", device);
	return -EINVAL;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
				   rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("BYT Comms Machine:  Set TDM Slot Fails %d\n", ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("BYT Comms Machine: Set Tristate Fails %d\n", ret);
	return -EINVAL;
	}

	pr_debug("BYT Comms Machine: slot_width = %d\n",
	     slot_width);
	pr_debug("BYT Comms Machine: tx_mask = %d\n",
	     tx_mask);
	pr_debug("BYT Comms Machine: rx_mask = %d\n",
	     rx_mask);
	pr_debug("BYT Comms Machine: tristate_offset = %d\n",
	     tristate_offset);

	return 0;
}

static int byt_comms_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	unsigned int device = substream->pcm->device;

	pr_debug("%s substream->runtime->rate %d\n",
		__func__,
		substream->runtime->rate);

	/* select clock source (if master) */
	/* BT SCO: CPU DAI is master */
	/* FM: CPU DAI is master */
	/* BT_VOIP: CPU DAI is master */
	if ((device == BYT_COMMS_BT && ctl->ssp_bt_sco_master_mode) ||
	    (device == BYT_COMMS_MODEM && ctl->ssp_modem_master_mode)) {

		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);

	}

	return 0;
}
//<asus-baron20131101->
static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);

	pr_debug("Enter:%s", __func__);
	/* Set codec bias level */
	byt_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE | SND_JACK_BTN_0,
			       &ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}
	ret = snd_soc_jack_add_gpios(&ctx->jack, 1, &hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}

	ret = snd_soc_add_card_controls(card, byt_mc_controls,
					ARRAY_SIZE(byt_mc_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}
//<asus-baron20131101+>
	/* Add Comms specific controls */
	ctx->comms_ctl.ssp_bt_sco_master_mode = false;
	ctx->comms_ctl.ssp_modem_master_mode = false;

	ret = snd_soc_add_card_controls(card, byt_ssp_comms_controls,
					ARRAY_SIZE(byt_ssp_comms_controls));

	if (ret) {
		pr_err("unable to add COMMS card controls\n");
		return ret;
	}
//<asus-baron20131101->
	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	/*TODO: CHECK this */
	snd_soc_dapm_ignore_suspend(dapm, "HPOL");
	snd_soc_dapm_ignore_suspend(dapm, "HPOR");

	snd_soc_dapm_ignore_suspend(dapm, "SPOLP");
	snd_soc_dapm_ignore_suspend(dapm, "SPOLN");
	snd_soc_dapm_ignore_suspend(dapm, "SPORP");
	snd_soc_dapm_ignore_suspend(dapm, "SPORN");

	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headphone");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Int Mic");

	snd_soc_dapm_sync(dapm);
	return ret;
}

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int byt_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops byt_aif1_ops = {
	.startup = byt_aif1_startup,
	.hw_params = byt_aif1_hw_params,
};
static struct snd_soc_ops byt_aif2_ops = {
	.hw_params = byt_aif2_hw_params,
};
//<asus-baron20131101+>
static struct snd_soc_ops byt_comms_dai_link_ops = {
	.startup = byt_comms_dai_link_startup,
	.hw_params = byt_comms_dai_link_hw_params,
	.prepare = byt_comms_dai_link_prepare,
};
//<asus-baron20131101->
static struct snd_soc_dai_link byt_dailink[] = {
	[BYT_AUD_AIF1] = {
		.name = "Baytrail Audio",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "rt5640-aif1",
		.codec_name = "rt5640.2-001c",
		.platform_name = "sst-platform",
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_aif1_ops,
		.playback_count = 2,
	},
	[BYT_AUD_AIF2] = {
		.name = "Baytrail Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "rt5640-aif2",
		.codec_name = "rt5640.2-001c",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &byt_aif2_ops,
	},
	//<asus-baron20131101+>
	[BYT_COMMS_BT] = {
		.name = "Baytrail Comms BT SCO",
		.stream_name = "BYT_BTSCO",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &byt_comms_dai_link_ops,
	},
	[BYT_COMMS_MODEM] = {
		.name = "Baytrail Comms MODEM",
		.stream_name = "BYT_MODEM_MIXING",
		.cpu_dai_name = SSP_MODEM_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &byt_comms_dai_link_ops,
	},
	//<asus-baron20131101->
};

#ifdef CONFIG_PM_SLEEP
static int snd_byt_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_byt_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
}

static int snd_byt_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_byt_prepare NULL
#define snd_byt_complete NULL
#define snd_byt_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_byt = {
	.name = "baytrailaudio",
	.dai_link = byt_dailink,
	.num_links = ARRAY_SIZE(byt_dailink),
	.set_bias_level = byt_set_bias_level,
	.dapm_widgets = byt_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(byt_dapm_widgets),
	.dapm_routes = byt_audio_map,
	.num_dapm_routes = ARRAY_SIZE(byt_audio_map),
};

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_mc_private *drv;
	struct byt_audio_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("Entry %s\n", __func__);

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	pdata->codec_gpio = acpi_get_gpio("\\_SB.GPO2", 4); /* GPIO_SUS4 */
	pdata->hsdet_gpio = acpi_get_gpio("\\_SB.GPO2", 28); /* GPIO_SUS28 */
	pdata->dock_hs_gpio = acpi_get_gpio("\\_SB.GPO2", 27); /* GPIO_SUS27 */
	pr_info("%s: GPIOs - codec %d, hsdet %d, dock_hs %d", __func__,
		pdata->codec_gpio, pdata->hsdet_gpio, pdata->dock_hs_gpio);

	hs_gpio.gpio = pdata->codec_gpio;
	/* register the soc card */
	snd_soc_card_byt.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_byt, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_byt);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_byt);
	pr_info("%s successful\n", __func__);
	return ret_val;
}

static int snd_byt_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_soc_jack_free_gpios(&drv->jack, 1, &hs_gpio);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_byt_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_soc_jack_free_gpios(&drv->jack, 1, &hs_gpio);
}

const struct dev_pm_ops snd_byt_mc_pm_ops = {
	.prepare = snd_byt_prepare,
	.complete = snd_byt_complete,
	.poweroff = snd_byt_poweroff,
};

static const struct acpi_device_id byt_mc_acpi_ids[] = {
	{ "AMCR0F28", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, byt_mc_acpi_ids);

static struct platform_driver snd_byt_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "byt_rt5642",
		.pm = &snd_byt_mc_pm_ops,
		.acpi_match_table = ACPI_PTR(byt_mc_acpi_ids),
	},
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
	.shutdown = snd_byt_mc_shutdown,
};

static int __init snd_byt_driver_init(void)
{
	pr_info("Baytrail Machine Driver byt_rt5642 registerd\n");
	return platform_driver_register(&snd_byt_mc_driver);
}
late_initcall(snd_byt_driver_init);

static void __exit snd_byt_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_byt_mc_driver);
}
module_exit(snd_byt_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail Machine driver");
MODULE_AUTHOR("Omair Md Abdullah <omair.m.abdullah@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bytrt5642-audio");
