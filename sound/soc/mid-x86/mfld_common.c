/*
 *  mfld_common.c - Common routines for the Medfield platform
 *  based on Intel Medfield MID platform
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
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

#include <linux/delay.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_scu_ipcutil.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include "../codecs/sn95031.h"
#include "mfld_common.h"

/* reads the voltage level from the ADC Driver*/
unsigned int mfld_jack_read_voltage(struct snd_soc_jack *jack)
{
	unsigned int mic_bias;
	struct mfld_mc_private *ctx =
			snd_soc_card_get_drvdata(jack->codec->card);

	/* Reads the mic bias value */
	if (!ctx->mfld_jack_lp_flag)
		/* GPADC MIC BIAS takes around a 50ms to settle down and
		* get sampled porperly, reading earlier than this causes to
		* read incorrect values */
		msleep(50);
	intel_mid_gpadc_sample(ctx->audio_adc_handle,
			MFLD_ADC_SAMPLE_COUNT, &mic_bias);
	mic_bias = (mic_bias * MFLD_ADC_ONE_LSB_MULTIPLIER) / 1000;
	pr_debug("mic bias = %dmV\n", mic_bias);
	return mic_bias;
}

int mfld_vibra_enable_clk(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	int clk_id = 0;

	if (!strcmp(w->name, "Vibra1Clock"))
		clk_id = CLK0_VIBRA1;
	else if (!strcmp(w->name, "Vibra2Clock"))
		clk_id = CLK0_VIBRA2;

	if (SND_SOC_DAPM_EVENT_ON(event))
		intel_scu_ipc_set_osc_clk0(true, clk_id);
	else if (SND_SOC_DAPM_EVENT_OFF(event))
		intel_scu_ipc_set_osc_clk0(false, clk_id);
	return 0;
}

/* Callback to set volume for *VOLCTRL regs. Needs to be implemented separately
 * since clock and VAUDA need to be on before value can be written to the regs
 */
int mfld_set_vol_2r(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	unsigned int val, val2, val_mask;
	int sst_pll_mode_saved;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (invert) {
		val = max - val;
		val2 = max - val2;
	}

	val = val << shift;
	val2 = val2 << shift;

	pr_debug("enabling PLL and VAUDA to change volume\n");
	mutex_lock(&codec->mutex);
	sst_pll_mode_saved = intel_scu_ipc_set_osc_clk0(true, CLK0_QUERY);
	intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
	snd_soc_dapm_force_enable_pin(&codec->dapm, "VirtBias");
	snd_soc_dapm_sync(&codec->dapm);

	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		goto restore_state;

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
restore_state:
	snd_soc_dapm_disable_pin(&codec->dapm, "VirtBias");
	if (!(sst_pll_mode_saved & CLK0_MSIC))
		intel_scu_ipc_set_osc_clk0(false, CLK0_MSIC);
	mutex_unlock(&codec->mutex);
	return err;
}

int mfld_get_pcm1_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	pr_debug("PCM1 master mode: %d\n", ctx->sn95031_pcm1_mode);
	ucontrol->value.integer.value[0] = ctx->sn95031_pcm1_mode;
	return 0;
}

int mfld_set_pcm1_mode(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	ctx->sn95031_pcm1_mode = ucontrol->value.integer.value[0];
	return 0;
}

int mfld_headset_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);
	ucontrol->value.integer.value[0] = ctx->hs_switch;
	return 0;
}

int mfld_headset_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	if (ucontrol->value.integer.value[0] == ctx->hs_switch)
		return 0;

	if (ucontrol->value.integer.value[0]) {
		pr_debug("hs_set HS path\n");
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
	} else {
		pr_debug("hs_set EP path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_enable_pin(&codec->dapm, "EPOUT");
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	ctx->hs_switch = ucontrol->value.integer.value[0];

	return 0;
}

static void mfld_lo_enable_out_pins(struct snd_soc_codec *codec)
{
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB1OUT");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB2OUT");
	if (ctx->hs_switch) {
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
	} else {
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_enable_pin(&codec->dapm, "EPOUT");
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
}

int mfld_lo_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);
	ucontrol->value.integer.value[0] = ctx->sn95031_lo_dac;
	return 0;
}

int mfld_lo_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);

	if (ucontrol->value.integer.value[0] == ctx->sn95031_lo_dac)
		return 0;

	/* we dont want to work with last state of lineout so just enable all
	 * pins and then disable pins not required
	 */
	mfld_lo_enable_out_pins(codec);
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		pr_debug("set vibra path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "VIB1OUT");
		snd_soc_dapm_disable_pin(&codec->dapm, "VIB2OUT");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0);
		break;

	case 1:
		pr_debug("set hs  path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x22);
		break;

	case 2:
		pr_debug("set spkr path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "IHFOUTL");
		snd_soc_dapm_disable_pin(&codec->dapm, "IHFOUTR");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x44);
		break;

	case 3:
		pr_debug("set null path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "LINEOUTL");
		snd_soc_dapm_disable_pin(&codec->dapm, "LINEOUTR");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x66);
		break;
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	ctx->sn95031_lo_dac = ucontrol->value.integer.value[0];
	return 0;
}

