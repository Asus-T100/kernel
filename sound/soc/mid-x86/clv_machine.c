/*
 *  clv_machine.c - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2011-12 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Vaibhav Agarwal <vaibhav.agarwal@intel.com>
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
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/rpmsg.h>
#include <linux/module.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/platform_clvs_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/cs42l73.h"
#include "ctp_common.h"
#include "clv_machine.h"

/* Headset jack detection gpios func(s) */
#define HPDETECT_POLL_INTERVAL	msecs_to_jiffies(1000)	/* 1sec */

/* CDB42L73 widgets */
static const struct snd_soc_dapm_widget clv_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", clv_amp_event),
};

/* CDB42L73 Audio Map */
static const struct snd_soc_dapm_route clv_audio_map[] = {
	{"MIC1", NULL, "Headset Mic"},
	{"MIC2", NULL, "Headset Mic"},
	/* Headphone (L+R)->  HPOUTA, HPOUTB */
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
	{"Ext Spk", NULL, "SPKLINEOUT"},
	{"Ext Spk", NULL, "SPKOUT"},
};


/* Board specific codec bias level control */
int clv_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (card->dapm.bias_level == SND_SOC_BIAS_OFF)
			intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
		card->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_OFF:
		/* OSC clk will be turned OFF after processing
		 * codec->dapm.bias_level = SND_SOC_BIAS_OFF.
		 */
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);

	return 0;
}

int clv_set_bias_level_post(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;
	/* we have only one codec in this machine */
	codec = list_entry(card->codec_dev_list.next, struct snd_soc_codec,
			card_list);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		/* Processing already done during set_bias_level()
		 * callback. No action required here.
		 */
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
			break;
		intel_scu_ipc_set_osc_clk0(false, CLK0_MSIC);
		card->dapm.bias_level = level;
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return 0;
}

static int get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;
	return 0;
}

static int set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_bt_sco_master_mode)
		return 0;

	ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];

	return 0;
}

static int get_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_voip_master_mode;
	return 0;
}

static int set_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_voip_master_mode)
		return 0;

	ctl->ssp_voip_master_mode = ucontrol->value.integer.value[0];

	return 0;
}


static int get_ssp_ifx_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_ifx_master_mode;
	return 0;
}

static int set_ssp_ifx_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_ifx_master_mode)
		return 0;

	ctl->ssp_ifx_master_mode = ucontrol->value.integer.value[0];

	return 0;
}

static const struct snd_kcontrol_new ssp_comms_controls[] = {
		SOC_ENUM_EXT("SSP BT Master Mode",
				ssp_bt_sco_master_mode_enum,
				get_ssp_bt_sco_master_mode,
				set_ssp_bt_sco_master_mode),
		SOC_ENUM_EXT("SSP VOIP Master Mode",
				ssp_voip_master_mode_enum,
				get_ssp_voip_master_mode,
				set_ssp_voip_master_mode),
		SOC_ENUM_EXT("SSP Modem Master Mode",
				ssp_ifx_master_mode_enum,
				get_ssp_ifx_master_mode,
				set_ssp_ifx_master_mode),
};

static int clv_asp_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	/* CS42L73  Slave Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
				| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);

	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
		DEFAULT_MCLK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	return 0;
}

static int clv_vsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret , clk_source;

	pr_debug("Slave Mode selected\n");
	/* CS42L73  Master Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	clk_source = SND_SOC_CLOCK_IN;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
		DEFAULT_MCLK, clk_source);
	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	return 0;
}
static int clv_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "CLV Comms Machine: ERROR "\
				"NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case CLV_COMMS_BT_SCO_DEV:
		str_runtime->hw = BT_sco_hw_param;
		break;

	case CLV_COMMS_MSIC_VOIP_DEV:
		str_runtime->hw = VOIP_alsa_hw_param;
		break;

	case CLV_COMMS_IFX_MODEM_DEV:
		str_runtime->hw = IFX_modem_alsa_hw_param;
		break;
	default:
		pr_err("MFLD Comms Machine: bad PCM Device = %d\n",
						substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
}

static int clv_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
	unsigned int device = substream->pcm->device;


	pr_debug("ssp_bt_sco_master_mode %d\n", ctl->ssp_bt_sco_master_mode);
	pr_debug("ssp_voip_master_mode %d\n", ctl->ssp_voip_master_mode);
	pr_debug("ssp_ifx_master_mode %d\n", ctl->ssp_ifx_master_mode);

	switch (device) {
	case CLV_COMMS_BT_SCO_DEV:
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
				SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("MFLD Comms Machine: Set FMT Fails %d\n",
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
		nb_slot = SSP_BT_SLOT_NB_SLOT;
		slot_width = SSP_BT_SLOT_WIDTH;
		tx_mask = SSP_BT_SLOT_TX_MASK;
		rx_mask = SSP_BT_SLOT_RX_MASK;

		if (ctl->ssp_bt_sco_master_mode)
			tristate_offset = BIT(TRISTATE_BIT);
		else
			tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		break;
	case CLV_COMMS_MSIC_VOIP_DEV:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_0
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_LOW
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_I2S |
				SSP_DAI_SCMODE_0 |
				SND_SOC_DAIFMT_NB_IF |
				(ctl->ssp_voip_master_mode ?
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

		tristate_offset = BIT(TRISTATE_BIT);
		break;

	case CLV_COMMS_IFX_MODEM_DEV:
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
				(ctl->ssp_ifx_master_mode ?
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

		tristate_offset = BIT(TRISTATE_BIT) |\
				BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);

		break;
	default:
		pr_err("CLV Comms Machine: bad PCM Device ID = %d\n",
				device);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
			rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("CLV Comms Machine:  Set TDM Slot Fails %d\n",
				ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("CLV Comms Machine: Set Tristate Fails %d\n",
				ret);
		return -EINVAL;
	}

	if (device == CLV_COMMS_MSIC_VOIP_DEV) {
		pr_debug("Call clv_vsp_hw_params to enable the PLL Codec\n");
		clv_vsp_hw_params(substream, params);
	}

	pr_debug("CLV Comms Machine: slot_width = %d\n",
			slot_width);
	pr_debug("CLV Comms Machine: tx_mask = %d\n",
			tx_mask);
	pr_debug("CLV Comms Machine: rx_mask = %d\n",
			rx_mask);
	pr_debug("CLV Comms Machine: tristate_offset = %d\n",
			tristate_offset);

	return 0;

} /* clv_comms_dai_link_hw_params*/

static int clv_comms_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	unsigned int device = substream->pcm->device;

	pr_debug("%s substream->runtime->rate %d\n",
			__func__,
			substream->runtime->rate);

	/* select clock source (if master) */
	/* BT SCO: CPU DAI is master */
	/* FM: CPU DAI is master */
	/* BT_VOIP: CPU DAI is master */
	if (((device == CLV_COMMS_BT_SCO_DEV &&\
		ctl->ssp_bt_sco_master_mode) ||
		((device == CLV_COMMS_MSIC_VOIP_DEV) &&\
		ctl->ssp_voip_master_mode)) ||
		(device == CLV_COMMS_IFX_MODEM_DEV &&\
		ctl->ssp_ifx_master_mode)) {

		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);

	}

	return 0;
} /* clv_comms_dai_link_prepare */

/* Headset jack detection gpios func(s) */
static int clv_soc_jack_gpio_detect(void);
static int clv_soc_jack_gpio_detect_bp(void);

static struct snd_soc_jack_gpio hs_gpio[] = {
	{
		.gpio = HPSENSE_GPIO,
		.name = "cs-hsdet-gpio",
		.report = SND_JACK_HEADSET,
		.debounce_time = 100,
		.jack_status_check = clv_soc_jack_gpio_detect,
		.irq_flags = IRQF_TRIGGER_FALLING,
	}, {
		.gpio = BUTTON_GPIO,
		.name = "cs-hsbutton-gpio",
		.report = SND_JACK_HEADSET | SND_JACK_BTN_0,
		.debounce_time = 100,
		.jack_status_check = clv_soc_jack_gpio_detect_bp,
		.irq_flags = IRQF_TRIGGER_FALLING,
	},
};

int clv_soc_jack_gpio_detect(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[0];
	int enable, status;
	int irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct clv_mc_private *ctx =
		container_of(jack, struct clv_mc_private, clv_jack);

	/* Get Jack status */
	gpio = &hs_gpio[0];
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);
	set_mic_bias(jack, MIC_BIAS_ENABLE);
	status = cs42l73_hp_detection(codec, jack, enable);
	if (!status) {
		set_mic_bias(jack, MIC_BIAS_DISABLE);
		/* Jack removed, Disable BP interrupts if not done already */
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			gpio = &hs_gpio[1];
			irq = gpio_to_irq(gpio->gpio);
			if (irq < 0) {
				pr_err("%d:Failed to map gpio_to_irq\n", irq);
				return status;
			}
			/* Disable Button_press interrupt if no Headset */
			pr_debug("Disable %d interrupt line\n", irq);
			disable_irq_nosync(irq);
		} else {
			atomic_inc(&ctx->bpirq_flag);
		}
	} else { /* If jack inserted, schedule delayed_wq */
		schedule_delayed_work(&ctx->jack_work, HPDETECT_POLL_INTERVAL);
#ifdef CONFIG_HAS_WAKELOCK
		/*
		 * Take wakelock for one second to give time for the detection
		 * to finish. Jack detection is happening rarely so this doesn't
		 * have big impact to power consumption.
		 */
		wake_lock_timeout(ctx->jack_wake_lock,
				HPDETECT_POLL_INTERVAL + msecs_to_jiffies(50));
#endif
	}
	return status;
}

/* Func to verify Jack status after HPDETECT_POLL_INTERVAL */
static void headset_status_verify(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[0];
	int enable, status, irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	unsigned int mask = SND_JACK_HEADSET;
	struct clv_mc_private *ctx =
		container_of(jack, struct clv_mc_private, clv_jack);

	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);

	status = cs42l73_hp_detection(codec, jack, enable);
	gpio = &hs_gpio[1];
	irq = gpio_to_irq(gpio->gpio);
	if (irq < 0) {
		pr_err("%d:Failed to map gpio_to_irq\n", irq);
		return;
	}

	/* Enable Button_press interrupt if HS is inserted
	 * and interrupts are not already enabled
	 */
	if (status == SND_JACK_HEADSET) {
		if (atomic_inc_return(&ctx->bpirq_flag) == 1) {
			/* If BP intr not enabled */
			pr_debug("Enable %d interrupt line\n", irq);
			enable_irq(irq);
		} else {
			atomic_dec(&ctx->bpirq_flag);
		}
		/* else do nothing as interrupts are already enabled
		 * This case occurs during slow insertion when
		 * multiple plug-in events are reported
		 */
	} else {
		set_mic_bias(jack, MIC_BIAS_DISABLE);
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			/* Disable Button_press interrupt if no Headset */
			pr_debug("Disable %d interrupt line\n", irq);
			disable_irq_nosync(irq);
		} else {
			atomic_inc(&ctx->bpirq_flag);
		}

	}

	if (jack->status != status)
		snd_soc_jack_report(jack, status, mask);

	pr_debug("%s: status 0x%x\n", __func__, status);
}

int clv_soc_jack_gpio_detect_bp(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[1];
	int enable, hs_status, status, irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct clv_mc_private *ctx =
		container_of(jack, struct clv_mc_private, clv_jack);

	status = 0;
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, enable);

	/* Check for headset status before processing interrupt */
	gpio = &hs_gpio[0];
	hs_status = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		hs_status = !hs_status;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, hs_status);
	if (!hs_status) {/* HS present, process the interrupt */
		if (!enable)
			status = cs42l73_bp_detection(codec, jack, enable);
		else {
			status = jack->status;
			pr_debug("%s:Invalid BP interrupt\n", __func__);
		}
	} else {
		pr_debug("%s:Spurious BP interrupt : HS_status 0x%x\n",
				__func__, hs_status);
		/* Disbale BP interrupts, in case enabled */
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			set_mic_bias(jack, MIC_BIAS_DISABLE);
			gpio = &hs_gpio[1];
			irq = gpio_to_irq(gpio->gpio);
			if (irq < 0) {
				pr_err("%d:Failed to map gpio_to_irq\n", irq);
				return status;
			}

			/* Disable Button_press interrupt if no Headset */
			pr_debug("Disable %d interrupt line\n", irq);
			disable_irq_nosync(irq);
		} else {
			atomic_inc(&ctx->bpirq_flag);
		}

	}

	pr_debug("%s: status 0x%x\n", __func__, status);

	return status;
}


static int clv_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret, irq;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_jack_gpio *gpio = &hs_gpio[1];
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);

	/* Set codec bias level */
	clv_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* Add Jack specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, clv_dapm_widgets,
					ARRAY_SIZE(clv_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Jack specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, clv_audio_map,
					ARRAY_SIZE(clv_audio_map));

	/* Add Comms specefic controls */
	ctx->comms_ctl.ssp_bt_sco_master_mode = false;
	ctx->comms_ctl.ssp_voip_master_mode = false;
	ctx->comms_ctl.ssp_ifx_master_mode = false;

	ret = snd_soc_add_card_controls(card, ssp_comms_controls,
				ARRAY_SIZE(ssp_comms_controls));
	if (ret) {
		pr_err("Add Comms Controls failed %d",
				ret);
		return ret;
	}


	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "EAROUT");
	snd_soc_dapm_ignore_suspend(dapm, "HPOUTA");
	snd_soc_dapm_ignore_suspend(dapm, "HPOUTB");
	snd_soc_dapm_ignore_suspend(dapm, "VSPOUT");

	/*In VV board SPKOUT is connected and SPKLINEOUT on PR board*/
	/*In VV board MIC1 is connected  and MIC2 is PR boards */
	if (ctp_vv_board()) {
		snd_soc_dapm_disable_pin(dapm, "MIC1");
		snd_soc_dapm_disable_pin(dapm, "SPKOUT");
		snd_soc_dapm_ignore_suspend(dapm, "SPKLINEOUT");
	} else {
		snd_soc_dapm_disable_pin(dapm, "MIC2");
		snd_soc_dapm_disable_pin(dapm, "SPKLINEOUT");
		snd_soc_dapm_ignore_suspend(dapm, "SPKOUT");
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);

	/* Setup the HPDET timer */
	INIT_DELAYED_WORK(&ctx->jack_work, headset_status_verify);

	/* Headset and button jack detection */
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0, &ctx->clv_jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	ret = snd_soc_jack_add_gpios(&ctx->clv_jack, 2, hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}
	irq = gpio_to_irq(gpio->gpio);
	if (irq < 0) {
		pr_err("%d:Failed to map gpio_to_irq\n", irq);
		return irq;
	}

	/* Disable Button_press interrupt if no Headset */
	pr_err("Disable %d interrupt line\n", irq);
	disable_irq_nosync(irq);
	atomic_set(&ctx->bpirq_flag, 0);

	return ret;
}

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count	= ARRAY_SIZE(rates_48000),
	.list	= rates_48000,
};


static int clv_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&constraints_48000);
	return 0;
}

static struct snd_soc_ops clv_asp_ops = {
	.startup = clv_startup,
	.hw_params = clv_asp_hw_params,
};

static struct snd_soc_ops clv_vsp_ops = {
	.hw_params = clv_vsp_hw_params,
};

static struct snd_soc_ops clv_comms_dai_link_ops = {
		.startup = clv_comms_dai_link_startup,
		.hw_params = clv_comms_dai_link_hw_params,
		.prepare = clv_comms_dai_link_prepare,
};
static struct snd_soc_ops clv_comms_voip_dai_link_ops = {
		.startup = clv_comms_dai_link_startup,
		.hw_params = clv_comms_dai_link_hw_params,
		.prepare = clv_comms_dai_link_prepare,
};

struct snd_soc_dai_link clv_msic_dailink[] = {
		[CLV_AUD_ASP_DEV] = {
		.name = "Cloverview ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = clv_init,
		.ignore_suspend = 1,
		.ops = &clv_asp_ops,
	},
	[CLV_AUD_VSP_DEV] = {
		.name = "Cloverview VSP",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-vsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &clv_vsp_ops,
	},

	[CLV_AUD_COMP_ASP_DEV] = {
		.name = "Cloverview Comp ASP",
		.stream_name = "Compress-Audio",
		.cpu_dai_name = "Virtual-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &clv_asp_ops,
	},
	[CLV_COMMS_BT_SCO_DEV] = {
		.name = "Cloverview Comms BT SCO",
		.stream_name = "BTSCO",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &clv_comms_dai_link_ops,
	},
	[CLV_COMMS_MSIC_VOIP_DEV] = {
		.name = "Cloverview Comms MSIC VOIP",
		.stream_name = "VOIP",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "cs42l73-vsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &clv_comms_voip_dai_link_ops,
	},
	[CLV_COMMS_IFX_MODEM_DEV] = {
		.name = "Cloverview Comms IFX MODEM",
		.stream_name = "IFX_MODEM_MIXING",
		.cpu_dai_name = SSP_MODEM_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &clv_comms_dai_link_ops,
	},

};

/* SoC card */
static struct snd_soc_card snd_soc_card_clv = {
	.name = "cloverview_audio",
	.dai_link = clv_msic_dailink,
	.num_links = ARRAY_SIZE(clv_msic_dailink),
	.set_bias_level = clv_set_bias_level,
	.set_bias_level_post = clv_set_bias_level_post,
};

int snd_clv_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct clv_mc_private *ctx;

	pr_debug("In %s\n", __func__);
	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
#ifdef CONFIG_HAS_WAKELOCK
	ctx->jack_wake_lock =
		kzalloc(sizeof(*(ctx->jack_wake_lock)), GFP_ATOMIC);
	if (!ctx->jack_wake_lock) {
		pr_err("allocation failed for wake_lock\n");
		kfree(ctx);
		return -ENOMEM;
	}
	wake_lock_init(ctx->jack_wake_lock, WAKE_LOCK_SUSPEND,
			"jack_detect");
#endif

	/* register the soc card */
	snd_soc_card_clv.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_clv, ctx);
	ret_val = snd_soc_register_card(&snd_soc_card_clv);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_clv);
	pr_debug("successfully exited probe\n");
	return ret_val;

unalloc:
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
	kfree(ctx->jack_wake_lock);
#endif
	kfree(ctx);
	return ret_val;
}

const struct dev_pm_ops snd_clv_mc_pm_ops = {
	.suspend = snd_clv_suspend,
	.resume = snd_clv_resume,
	.poweroff = snd_clv_poweroff,
};

static struct platform_driver snd_clv_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "clvcs_audio",
		.pm   = &snd_clv_mc_pm_ops,
	},
	.probe = snd_clv_mc_probe,
	.remove = __devexit_p(snd_clv_mc_remove),
};

static int __init snd_clv_driver_init(void)
{
	pr_info("In %s\n", __func__);
	return platform_driver_register(&snd_clv_mc_driver);
}

static void __exit snd_clv_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_clv_mc_driver);
}

static int snd_clv_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_clv rpmsg device\n");

	ret = snd_clv_driver_init();

out:
	return ret;
}

static void __devexit snd_clv_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_clv_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_clv rpmsg device\n");
}

static void snd_clv_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id snd_clv_rpmsg_id_table[] = {
	{ .name	= "rpmsg_msic_clv_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_clv_rpmsg_id_table);

static struct rpmsg_driver snd_clv_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_clv_rpmsg_id_table,
	.probe		= snd_clv_rpmsg_probe,
	.callback	= snd_clv_rpmsg_cb,
	.remove		= __devexit_p(snd_clv_rpmsg_remove),
};

static int __init snd_clv_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_clv_rpmsg);
}

late_initcall(snd_clv_rpmsg_init);

static void __exit snd_clv_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_clv_rpmsg);
}
module_exit(snd_clv_rpmsg_exit);


MODULE_DESCRIPTION("ASoC Intel(R) Cloverview MID Machine driver");
MODULE_AUTHOR("Jeeja KP<jeeja.kp@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:clvcs42l73-audio");
