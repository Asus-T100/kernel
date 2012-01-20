/*
 *  mfld_machine.c - ASoc Machine driver for Intel Medfield MID platform
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
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/ipc_device.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/tlv.h>
#include "../codecs/sn95031.h"

#define MFLD_JACK_INSERT 0x04
#define HEADSET_DET_PIN 77

enum soc_mic_bias_zones {
	MFLD_MV_START = 0,
	/* mic bias volutage range for Headphones*/
	MFLD_MV_HP = 400,
	/* mic bias volutage range for American Headset*/
	MFLD_MV_AM_HS = 650,
	/* mic bias volutage range for Headset*/
	MFLD_MV_HS = 2000,
	MFLD_MV_UNDEFINED,
};

struct mfld_mc_private {
	void __iomem *int_base;
	u8 jack_interrupt_status;
	u8 oc_interrupt_status;
	spinlock_t lock; /* lock for interrupt status and jack debounce */
	int pcm1_master_mode;
	unsigned int hs_switch;
	unsigned int lo_dac;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock wake_lock;
#endif
};

static struct snd_soc_jack mfld_jack;

/* jack detection voltage zones */
static struct snd_soc_jack_zone mfld_zones[] = {
	{MFLD_MV_START, MFLD_MV_AM_HS, SND_JACK_HEADPHONE},
	{MFLD_MV_AM_HS, MFLD_MV_HS, SND_JACK_HEADSET},
};

/* sound card controls */
static const char *headset_switch_text[] = {"Earpiece", "Headset"};

static const char *lo_text[] = {"Vibra", "Headset", "IHF", "None"};

static const struct soc_enum headset_enum =
	SOC_ENUM_SINGLE_EXT(2, headset_switch_text);

static const struct soc_enum lo_enum =
	SOC_ENUM_SINGLE_EXT(4, lo_text);

static const char *sn95031_pcm1_mode_text[] = {"Slave", "Master"};

static const struct soc_enum SN95031_pcm1_mode_config_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sn95031_pcm1_mode_text),
							sn95031_pcm1_mode_text);

static int headset_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);
	ucontrol->value.integer.value[0] = mc_drv_ctx->hs_switch;
	return 0;
}

static int headset_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);

	if (ucontrol->value.integer.value[0] == mc_drv_ctx->hs_switch)
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
	mc_drv_ctx->hs_switch = ucontrol->value.integer.value[0];

	return 0;
}

static void lo_enable_out_pins(struct snd_soc_codec *codec)
{
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);
	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB1OUT");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB2OUT");
	if (mc_drv_ctx->hs_switch) {
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
	} else {
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_enable_pin(&codec->dapm, "EPOUT");
	}
}

static int lo_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);
	ucontrol->value.integer.value[0] = mc_drv_ctx->lo_dac;
	return 0;
}

static int lo_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);

	if (ucontrol->value.integer.value[0] == mc_drv_ctx->lo_dac)
		return 0;

	/* we dont want to work with last state of lineout so just enable all
	 * pins and then disable pins not required
	 */
	lo_enable_out_pins(codec);
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
	mc_drv_ctx->lo_dac = ucontrol->value.integer.value[0];
	return 0;
}

static int sn95031_get_pcm1_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);

	pr_debug("PCM1 master mode: %d\n", mc_drv_ctx->pcm1_master_mode);
	ucontrol->value.integer.value[0] = mc_drv_ctx->pcm1_master_mode;
	return 0;
}

static int sn95031_set_pcm1_mode(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);

	mc_drv_ctx->pcm1_master_mode = ucontrol->value.integer.value[0];
	return 0;
}

static int mfld_vibra_enable_clk(struct snd_soc_dapm_widget *w,
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
static int sn95031_set_vol_2r(struct snd_kcontrol *kcontrol,
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

static const DECLARE_TLV_DB_SCALE(out_tlv, -6200, 100, 0);

static const struct snd_kcontrol_new mfld_snd_controls[] = {
	SOC_ENUM_EXT("Playback Switch", headset_enum,
			headset_get_switch, headset_set_switch),
	SOC_ENUM_EXT("Lineout Mux", lo_enum,
			lo_get_switch, lo_set_switch),
	SOC_ENUM_EXT("PCM1 Mode", SN95031_pcm1_mode_config_enum,
			sn95031_get_pcm1_mode, sn95031_set_pcm1_mode),
	/* Add digital volume and mute controls for Headphone/Headset*/
	SOC_DOUBLE_R_EXT_TLV("Headphone Playback Volume", SN95031_HSLVOLCTRL,
				SN95031_HSRVOLCTRL, 0, 71, 1,
				snd_soc_get_volsw_2r, sn95031_set_vol_2r,
				out_tlv),
	SOC_DOUBLE_R_EXT_TLV("Speaker Playback Volume", SN95031_IHFLVOLCTRL,
				SN95031_IHFRVOLCTRL, 0, 71, 1,
				snd_soc_get_volsw_2r, sn95031_set_vol_2r,
				out_tlv),
};

static const struct snd_soc_dapm_widget mfld_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
	/* Dummy widget to trigger VAUDA on/off */
	SND_SOC_DAPM_MICBIAS("VirtBias", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("Vibra1Clock", SND_SOC_NOPM, 0, 0,
			mfld_vibra_enable_clk,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("Vibra2Clock", SND_SOC_NOPM, 0, 0,
			mfld_vibra_enable_clk,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route mfld_map[] = {
	{ "HPOUTL", NULL, "Headset Rail"},
	{ "HPOUTR", NULL, "Headset Rail"},
	{"Headphones", NULL, "HPOUTR"},
	{"Headphones", NULL, "HPOUTL"},
	{"AMIC1", NULL, "Mic"},
	{"VIB1SPI", NULL, "Vibra1Clock"},
	{"VIB2SPI", NULL, "Vibra2Clock"},
};

static void mfld_jack_check(unsigned int intr_status)
{
	struct mfld_jack_data jack_data;

	jack_data.mfld_jack = &mfld_jack;
	jack_data.intr_id = intr_status;

	sn95031_jack_detection(&jack_data);
	/* TODO: add american headset detection post gpiolib support */
}

static unsigned int async_param;
static LIST_HEAD(mfld_jack_async_list);
static void mfld_jack_check_async(void *ptr, async_cookie_t cookie)
{
	mfld_jack_check(*(unsigned int *)ptr);
	return;
}

static int mfld_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct mfld_mc_private *mc_drv_ctx =
			snd_soc_card_get_drvdata(codec->card);
	int ret_val;

	/* Add jack sense widgets */
	snd_soc_dapm_new_controls(dapm, mfld_widgets, ARRAY_SIZE(mfld_widgets));

	/* Set up the map */
	snd_soc_dapm_add_routes(dapm, mfld_map, ARRAY_SIZE(mfld_map));

	ret_val = snd_soc_add_controls(codec, mfld_snd_controls,
				ARRAY_SIZE(mfld_snd_controls));
	if (ret_val) {
		pr_err("soc_add_controls failed %d", ret_val);
		return ret_val;
	}
	/* default is earpiece pin, userspace sets it explcitly */
	snd_soc_dapm_disable_pin(dapm, "Headphones");
	/* default is lineout NC, userspace sets it explcitly */
	snd_soc_dapm_disable_pin(dapm, "LINEOUTL");
	snd_soc_dapm_disable_pin(dapm, "LINEOUTR");
	mc_drv_ctx->lo_dac = 3;
	mc_drv_ctx->hs_switch = 0;
	/* we dont use linein in this so set to NC */
	snd_soc_dapm_disable_pin(dapm, "LINEINL");
	snd_soc_dapm_disable_pin(dapm, "LINEINR");
	snd_soc_dapm_disable_pin(dapm, "DMIC2");
	snd_soc_dapm_disable_pin(dapm, "DMIC3");
	snd_soc_dapm_disable_pin(dapm, "DMIC4");
	snd_soc_dapm_disable_pin(dapm, "DMIC6");
	snd_soc_dapm_disable_pin(dapm, "AMIC2");

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "PCM1_IN");
	snd_soc_dapm_ignore_suspend(dapm, "PCM1_Out");
	snd_soc_dapm_ignore_suspend(dapm, "EPOUT");
	snd_soc_dapm_ignore_suspend(dapm, "IHFOUTL");
	snd_soc_dapm_ignore_suspend(dapm, "IHFOUTR");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "Headphones");
	snd_soc_dapm_ignore_suspend(dapm, "Mic");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);
	/* Headset and button jack detection */
	ret_val = snd_soc_jack_new(codec, "Intel(R) MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0 |
			SND_JACK_BTN_1, &mfld_jack);
	if (ret_val) {
		pr_err("jack creation failed\n");
		return ret_val;
	}

	ret_val = snd_soc_jack_add_zones(&mfld_jack,
			ARRAY_SIZE(mfld_zones), mfld_zones);
	if (ret_val) {
		pr_err("adding jack zones failed\n");
		return ret_val;
	}

	/* we want to check if anything is inserted at boot,
	 * so send a fake event to codec and it will read adc
	 * to find if anything is there or not */
	async_param = MFLD_JACK_INSERT;
	async_schedule_domain(mfld_jack_check_async,
			&async_param, &mfld_jack_async_list);
	return ret_val;
}

#ifdef CONFIG_SND_MFLD_MONO_SPEAKER_SUPPORT
static int mfld_speaker_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_dai *cpu_dai = runtime->cpu_dai;
	struct snd_soc_dapm_context *dapm = &runtime->codec->dapm;

	snd_soc_dapm_disable_pin(dapm, "IHFOUTR");
	snd_soc_dapm_sync(dapm);
	return cpu_dai->driver->ops->set_tdm_slot(cpu_dai, 0, 0, 1, 0);
}
#endif

static int mfld_media_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	pr_debug("%s\n", __func__);
	/* Force the data width to 24 bit in MSIC since post processing
	algorithms in DSP enabled with 24 bit precision */
	ret = snd_soc_codec_set_params(codec, SNDRV_PCM_FORMAT_S24_LE);
	if (ret < 0) {
		pr_debug("codec_set_params returned error %d\n", ret);
		return ret;
	}
	snd_soc_codec_set_pll(codec, 0, SN95031_PLLIN, 1, 1);

	/* VAUD needs to be on before configuring PLL */
	snd_soc_dapm_force_enable_pin(&codec->dapm, "VirtBias");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	usleep_range(5000, 6000);
	sn95031_configure_pll(codec, ENABLE_PLL);

	/* enable PCM2 */
	snd_soc_dai_set_tristate(rtd->codec_dai, 0);
	return 0;
}

static int mfld_voice_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = rtd->card;
	struct mfld_mc_private *mc_drv_ctx = snd_soc_card_get_drvdata(soc_card);
	pr_debug("%s\n", __func__);

	if (mc_drv_ctx->pcm1_master_mode) { /* VOIP call */
		snd_soc_codec_set_pll(codec, 0, SN95031_PLLIN, 1, 1);
		snd_soc_dai_set_fmt(rtd->codec_dai, SND_SOC_DAIFMT_CBM_CFM
						| SND_SOC_DAIFMT_DSP_A);
		/* Sets the PCM1 clock rate */
		snd_soc_update_bits(codec, SN95031_PCM1C1, BIT(0)|BIT(1),
								BIT(0)|BIT(1));
	} else { /* CSV call */
		snd_soc_codec_set_pll(codec, 0, SN95031_PCM1BCLK, 1, 1);
		snd_soc_dai_set_fmt(rtd->codec_dai, SND_SOC_DAIFMT_CBS_CFS
						| SND_SOC_DAIFMT_I2S);
		snd_soc_update_bits(codec, SN95031_PCM1C1, BIT(0)|BIT(1), 0);
	}

	/* VAUD needs to be on before configuring PLL */
	snd_soc_dapm_force_enable_pin(&codec->dapm, "VirtBias");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
	usleep_range(5000, 6000);
	sn95031_configure_pll(codec, ENABLE_PLL);
	return 0;
}

static unsigned int rates_44100[] = {
	44100,
};

static struct snd_pcm_hw_constraint_list constraints_44100 = {
	.count	= ARRAY_SIZE(rates_44100),
	.list	= rates_44100,
};

static int mfld_media_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_44100);
	intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
	return 0;
}

static void mfld_media_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	pr_debug("%s\n", __func__);

	snd_soc_dapm_disable_pin(&rtd->codec->dapm, "VirtBias");
	/* switch off PCM2 port */
	if (!rtd->codec->active)
		snd_soc_dai_set_tristate(codec_dai, 1);
}

static int mfld_voice_startup(struct snd_pcm_substream *substream)
{
	intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
	return 0;
}

static void mfld_voice_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	pr_debug("%s\n", __func__);

	snd_soc_dapm_disable_pin(&rtd->codec->dapm, "VirtBias");
}

static struct snd_soc_ops mfld_media_ops = {
	.startup = mfld_media_startup,
	.shutdown = mfld_media_shutdown,
	.hw_params = mfld_media_hw_params,
};

static struct snd_soc_ops mfld_voice_ops = {
	.startup = mfld_voice_startup,
	.shutdown = mfld_voice_shutdown,
	.hw_params = mfld_voice_hw_params,
};

static struct snd_soc_dai_link mfld_msic_dailink[] = {
	{
		.name = "Medfield Headset",
		.stream_name = "Headset",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "SN95031 Headset",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = mfld_init,
		.ignore_suspend = 1,
		.ops = &mfld_media_ops,
	},
	{
		.name = "Medfield Speaker",
		.stream_name = "Speaker",
		.cpu_dai_name = "Speaker-cpu-dai",
		.codec_dai_name = "SN95031 Speaker",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
#ifdef CONFIG_SND_MFLD_MONO_SPEAKER_SUPPORT
		.init = mfld_speaker_init,
#else
		.init = NULL,
#endif
		.ignore_suspend = 1,
		.ops = &mfld_media_ops,
	},
/*
 *	This configurtaion doesnt need Vibra as PCM device
 *	so comment this out.
 *	codec should have SPI controls
 */
/*	{
		.name = "Medfield Vibra",
		.stream_name = "Vibra1",
		.cpu_dai_name = "Vibra1-cpu-dai",
		.codec_dai_name = "SN95031 Vibra1",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
	},
	{
		.name = "Medfield Haptics",
		.stream_name = "Vibra2",
		.cpu_dai_name = "Vibra2-cpu-dai",
		.codec_dai_name = "SN95031 Vibra2",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
	},
*/	{
		.name = "Medfield Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "SN95031 Voice",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.ops = &mfld_voice_ops,
	},
};

#ifdef CONFIG_PM

static int snd_mfld_mc_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
static int snd_mfld_mc_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

static int snd_mfld_mc_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_mfld_mc_suspend NULL
#define snd_mfld_mc_resume NULL
#define snd_mfld_mc_poweroff NULL
#endif

static int mfld_card_stream_event(struct snd_soc_dapm_context *dapm, int event)
{
	struct snd_soc_codec *codec = dapm->codec;
	pr_debug("machine stream event: %d\n", event);
	if (event == SND_SOC_DAPM_STREAM_STOP) {
		if (!codec->active) {
			sn95031_configure_pll(codec, DISABLE_PLL);
			return intel_scu_ipc_set_osc_clk0(false, CLK0_MSIC);
		}
	}
	return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_mfld = {
	.name = "medfield_audio",
	.dai_link = mfld_msic_dailink,
	.num_links = ARRAY_SIZE(mfld_msic_dailink),
};

static irqreturn_t snd_mfld_jack_intr_handler(int irq, void *dev)
{
	struct mfld_mc_private *mc_private = (struct mfld_mc_private *) dev;
	u16 intr_status = 0;

	memcpy_fromio(&intr_status, ((void *)(mc_private->int_base)),
			sizeof(u16));
	/* not overwrite status here */
	spin_lock(&mc_private->lock);
	/*To retrieve the jack_interrupt_status value (MSB)*/
	mc_private->jack_interrupt_status |= 0x0F & (intr_status >> 8);
	/*To retrieve the oc_interrupt_status value (LSB)*/
	mc_private->oc_interrupt_status |= 0x1F & intr_status;
	spin_unlock(&mc_private->lock);
#ifdef CONFIG_HAS_WAKELOCK
	/*
	 * We don't have any call back from the jack detection completed.
	 * Take wakelock for two seconds to give time for the detection
	 * to finish. Jack detection is happening rarely so this doesn't
	 * have big impact to power consumption.
	 */
	wake_lock_timeout(&mc_private->wake_lock, 2*HZ);
#endif
	return IRQ_WAKE_THREAD;
}

static irqreturn_t snd_mfld_codec_intr_detection(int irq, void *data)
{
	struct mfld_mc_private *mc_drv_ctx = (struct mfld_mc_private *) data;
	unsigned long flags;
	u8 jack_int_value = 0;

	if (mfld_jack.codec == NULL) {
		pr_debug("codec NULL returning..");
		spin_lock_irqsave(&mc_drv_ctx->lock, flags);
		mc_drv_ctx->jack_interrupt_status = 0;
		mc_drv_ctx->oc_interrupt_status = 0;
		spin_unlock_irqrestore(&mc_drv_ctx->lock, flags);
		goto ret;
	}
	spin_lock_irqsave(&mc_drv_ctx->lock, flags);
	if (!((mc_drv_ctx->jack_interrupt_status) ||
			(mc_drv_ctx->oc_interrupt_status))) {
		spin_unlock_irqrestore(&mc_drv_ctx->lock, flags);
		pr_err("OC and Jack Intr with status 0, return....\n");
		goto ret;
	}
	if (mc_drv_ctx->oc_interrupt_status) {
		pr_info("OC int value: %d\n", mc_drv_ctx->oc_interrupt_status);
		mc_drv_ctx->oc_interrupt_status = 0;
	}
	if (mc_drv_ctx->jack_interrupt_status) {
		jack_int_value = mc_drv_ctx->jack_interrupt_status;
		mc_drv_ctx->jack_interrupt_status = 0;
	}
	spin_unlock_irqrestore(&mc_drv_ctx->lock, flags);

	if (jack_int_value)
		mfld_jack_check(jack_int_value);

ret:
	return IRQ_HANDLED;
}

static int __devinit snd_mfld_mc_probe(struct ipc_device *ipcdev)
{
	int ret_val = 0, irq;
	struct mfld_mc_private *mc_drv_ctx;
	struct resource *irq_mem;

	pr_debug("snd_mfld_mc_probe called\n");

	/* retrive the irq number */
	irq = ipc_get_irq(ipcdev, 0);

	/* audio interrupt base of SRAM location where
	 * interrupts are stored by System FW */
	mc_drv_ctx = kzalloc(sizeof(*mc_drv_ctx), GFP_ATOMIC);
	if (!mc_drv_ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	spin_lock_init(&mc_drv_ctx->lock);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&mc_drv_ctx->wake_lock,
		       WAKE_LOCK_SUSPEND, "jack_detect");
#endif

	irq_mem = ipc_get_resource_byname(
				ipcdev, IORESOURCE_MEM, "IRQ_BASE");
	if (!irq_mem) {
		pr_err("no mem resource given\n");
		ret_val = -ENODEV;
		goto unalloc;
	}

	if (mfld_board_id() == MFLD_BID_PR3) {
		ret_val = gpio_request(HEADSET_DET_PIN, "headset_detect_pin");
		if (ret_val) {
			pr_err("HEADSET GPIO allocation failed: %d\n", ret_val);
			kfree(mc_drv_ctx);
			return ret_val;
		}
		ret_val = gpio_direction_input(HEADSET_DET_PIN);
		if (ret_val) {
			pr_err("HEADSET GPIO direction wrong: %d\n", ret_val);
			kfree(mc_drv_ctx);
			return ret_val;
		}
	}

	mc_drv_ctx->int_base = ioremap_nocache(irq_mem->start,
					resource_size(irq_mem));
	if (!mc_drv_ctx->int_base) {
		pr_err("Mapping of cache failed\n");
		ret_val = -ENOMEM;
		goto unalloc;
	}
	/* register for interrupt */
	ret_val = request_threaded_irq(irq, snd_mfld_jack_intr_handler,
			snd_mfld_codec_intr_detection,
			IRQF_SHARED | IRQF_NO_SUSPEND,
			ipcdev->dev.driver->name, mc_drv_ctx);
	if (ret_val) {
		pr_err("cannot register IRQ\n");
		goto unalloc;
	}
	/* register the soc card */
	snd_soc_card_mfld.dev = &ipcdev->dev;
	snd_soc_card_mfld.dapm.stream_event = mfld_card_stream_event;
	snd_soc_card_set_drvdata(&snd_soc_card_mfld, mc_drv_ctx);
	ret_val = snd_soc_register_card(&snd_soc_card_mfld);
	if (ret_val) {
		pr_debug("snd_soc_register_card failed %d\n", ret_val);
		goto freeirq;
	}
	ipc_set_drvdata(ipcdev, &snd_soc_card_mfld);
	pr_debug("successfully exited probe\n");
	return ret_val;

freeirq:
	free_irq(irq, mc_drv_ctx);
unalloc:
	kfree(mc_drv_ctx);
	return ret_val;
}

static int __devexit snd_mfld_mc_remove(struct ipc_device *ipcdev)
{
	struct snd_soc_card *soc_card = ipc_get_drvdata(ipcdev);
	struct mfld_mc_private *mc_drv_ctx = snd_soc_card_get_drvdata(soc_card);
	pr_debug("snd_mfld_mc_remove called\n");
	free_irq(ipc_get_irq(ipcdev, 0), mc_drv_ctx);
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(&mc_drv_ctx->wake_lock))
		wake_unlock(&mc_drv_ctx->wake_lock);
	wake_lock_destroy(&mc_drv_ctx->wake_lock);
#endif
	kfree(mc_drv_ctx);
	if (mfld_board_id() == MFLD_BID_PR3)
		gpio_free(HEADSET_DET_PIN);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	ipc_set_drvdata(ipcdev, NULL);
	return 0;
}
static const struct dev_pm_ops snd_mfld_mc_pm_ops = {
	.suspend = snd_mfld_mc_suspend,
	.resume = snd_mfld_mc_resume,
	.poweroff = snd_mfld_mc_poweroff,
};

static struct ipc_driver snd_mfld_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "msic_audio",
		.pm   = &snd_mfld_mc_pm_ops,
	},
	.probe = snd_mfld_mc_probe,
	.remove = __devexit_p(snd_mfld_mc_remove),
};

static int __init snd_mfld_driver_init(void)
{
	pr_info("snd_mfld_driver_init called\n");
	return ipc_driver_register(&snd_mfld_mc_driver);
}
late_initcall(snd_mfld_driver_init);

static void __exit snd_mfld_driver_exit(void)
{
	pr_debug("snd_mfld_driver_exit called\n");
	async_synchronize_full_domain(&mfld_jack_async_list);
	ipc_driver_unregister(&snd_mfld_mc_driver);
}
module_exit(snd_mfld_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ipc:msic-audio");
