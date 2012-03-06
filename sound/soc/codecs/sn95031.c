/*
 *  sn95031.c -  TI sn95031 Codec driver
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
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel-mid.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include <sound/intel_sst.h>
#include "sn95031.h"

#define SN95031_RATES (SNDRV_PCM_RATE_8000_96000)
#define SN95031_FORMATS (SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE)
#define SN95031_SW_DBNC 250
#define LP_THRESHOLD 400
#define HEADSET_DET_PIN 77

struct sn95031_jack_work {
	unsigned int intr_id;
	struct delayed_work work;
	struct snd_soc_jack *jack;
};

/* codec private data */
struct sn95031_priv {
	uint8_t clk_src;
	enum sn95031_pll_status  pll_state;
	struct sn95031_jack_work jack_work;
};

void *audio_adc_handle;
unsigned int sn95031_lp_flag;

/* This Function reads the voltage level from the ADC Driver*/
static unsigned int sn95031_read_voltage(void)
{
	unsigned int mic_bias;

	/* Reads the mic bias value */
	if (!sn95031_lp_flag)
		/* GPADC MIC BIAS takes around a 50ms to settle down and
		* get sampled porperly, reading earlier than this causes to
		* read incorrect values */
		msleep(50);
	intel_mid_gpadc_sample(audio_adc_handle, SN95031_ADC_SAMPLE_COUNT,
								&mic_bias);
	mic_bias = (mic_bias * SN95031_ADC_ONE_LSB_MULTIPLIER) / 1000;
	pr_debug("mic bias = %dmV\n", mic_bias);
	return mic_bias;
}

/* enables mic bias voltage */
static void sn95031_enable_mic_bias(struct snd_soc_codec *codec)
{
	pr_debug("enable mic bias\n");
	pr_debug("codec %p\n", codec);
	mutex_lock(&codec->mutex);
	snd_soc_dapm_force_enable_pin(&codec->dapm, "AMIC1Bias");
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
}

/* disables mic bias voltage */
static void sn95031_disable_mic_bias(struct snd_soc_codec *codec)
{
	pr_debug("disable mic bias\n");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_disable_pin(&codec->dapm, "AMIC1Bias");
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);
}
/* end - adc helper functions */

static inline unsigned int sn95031_read(struct snd_soc_codec *codec,
			unsigned int reg)
{
	u8 value = 0;
	int ret;

	ret = intel_scu_ipc_ioread8(reg, &value);
	if (ret)
		pr_err("read of %x failed, err %d\n", reg, ret);
	return value;

}

static inline int sn95031_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	int ret;

	ret = intel_scu_ipc_iowrite8(reg, value);
	if (ret)
		pr_err("write of %x failed, err %d\n", reg, ret);
	return ret;
}

static void sn95031_configure_pll(struct snd_soc_codec *codec, int operation)
{
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec);

	if (operation) {
		pr_debug("enabling PLL\n");
		/* PLL takes few msec to stabilize
		Refer sec2.3 MFLD Audio Interface Doc-rev0.7 */
		snd_soc_write(codec, SN95031_AUDPLLCTRL,
					(sn95031_ctx->clk_src)<<2);
		udelay(1000);
		snd_soc_update_bits(codec, SN95031_AUDPLLCTRL, BIT(1), BIT(1));
		udelay(1000);
		snd_soc_update_bits(codec, SN95031_AUDPLLCTRL, BIT(5), BIT(5));
		udelay(1000);
		sn95031_ctx->pll_state = PLL_ENABLED;
	} else {
		pr_debug("disabling PLL\n");
		sn95031_ctx->clk_src = SN95031_INVALID;
		sn95031_ctx->pll_state = PLL_DISABLED;
		snd_soc_write(codec, SN95031_AUDPLLCTRL, 0);
	}
}

static int sn95031_codec_stream_event(struct snd_soc_dapm_context *dapm,
		int event)
{
	pr_debug("%s:Event=%d\n", __func__, event);

	if (event == SND_SOC_DAPM_STREAM_STOP) {
		/* disable the MSIC PLL only if no other active streams */
		if (dapm->codec->active == 0) {
			sn95031_configure_pll(dapm->codec, DISABLE_PLL);
			/* disable PLLIN source clock */
			intel_sst_set_pll(false, SST_PLL_MSIC);
		}
	}
	return 0;
}
static int sn95031_set_vaud_bias(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		pr_debug("bias_prepare\n");
		/* enable msic pll only when any codec dai is active,
			not other use cases like static vibra etc */
		if (codec->active) {
			pr_debug("vaud_bias powering up pll\n");
			intel_sst_set_pll(true, SST_PLL_MSIC);
			/* allow few ms to stabilize the clock before
				enabling the MSIC PLL */
			usleep_range(5000, 6000);
			sn95031_configure_pll(codec, ENABLE_PLL);
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			pr_debug("vaud_bias power up rail\n");
			/* power up the rail, on in normal and aoac mode */
			snd_soc_write(codec, SN95031_VAUD, 0x2D);
			msleep(1);
		} else if (codec->dapm.bias_level == SND_SOC_BIAS_PREPARE) {
			pr_debug("vaud_bias standby\n");
		}
		break;

	case SND_SOC_BIAS_OFF:
		pr_debug("vaud_bias _OFF doing rail shutdown\n");
		/*
		 * off mode is 100, and we need AOAC as off as well,
		 * so 100100b ie 24
		 */
		snd_soc_write(codec, SN95031_VAUD, 0x24);
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

static int sn95031_vhs_event(struct snd_soc_dapm_widget *w,
		    struct snd_kcontrol *kcontrol, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("VHS SND_SOC_DAPM_EVENT_ON doing rail startup now\n");
		/* power up the rail- 1.8v, powersave mode */
		snd_soc_write(w->codec, SN95031_VHSP, 0xED);
		snd_soc_write(w->codec, SN95031_VHSN, 0x2D);
		msleep(1);
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		pr_debug("VHS SND_SOC_DAPM_EVENT_OFF doing rail shutdown\n");
		/* First disable VHSN and then followed by VHSP rail.
		Need to have minimum of 5ms delay between the rail shutdowns
		to avoid any glitches due to transients. */
		snd_soc_write(w->codec, SN95031_VHSN, 0x24);
		usleep_range(5000, 6000);
		snd_soc_write(w->codec, SN95031_VHSP, 0x24);
	}
	return 0;
}

static int sn95031_vihf_event(struct snd_soc_dapm_widget *w,
		    struct snd_kcontrol *kcontrol, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("VIHF SND_SOC_DAPM_EVENT_ON doing rail startup now\n");
		/* power up the rail */
		snd_soc_write(w->codec, SN95031_VIHF, 0x2D);
		msleep(1);
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		pr_debug("VIHF SND_SOC_DAPM_EVENT_OFF doing rail shutdown\n");
		snd_soc_write(w->codec, SN95031_VIHF, 0x24);
	}
	return 0;
}

static int sn95031_dmic12_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	unsigned int ldo = 0, clk_dir = 0, data_dir = 0, bias = 0;

	pr_debug("sn95031_dmic12_event\n");
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("sn95031_dmic12_event ON\n");
		ldo = BIT(5)|BIT(4);
		clk_dir = BIT(0);
		data_dir = BIT(7);
		bias = BIT(3);
	}
	/* program DMIC LDO, clock and set clock */
	snd_soc_update_bits(w->codec, SN95031_MICBIAS, BIT(5)|BIT(4), ldo);
	snd_soc_update_bits(w->codec, SN95031_DMICBUF0123, BIT(0), clk_dir);
	snd_soc_update_bits(w->codec, SN95031_DMICBUF0123, BIT(7), data_dir);
	snd_soc_update_bits(w->codec, SN95031_DMICMUX, BIT(3), bias);
	return 0;
}

static int sn95031_dmic34_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	unsigned int ldo = 0, clk_dir = 0, data_dir = 0, bias = 0;

	pr_debug("sn95031_dmic34_event\n");
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("sn95031_dmic34_event ON\n");
		ldo = BIT(5)|BIT(4);
		clk_dir = BIT(2);
		data_dir = BIT(1);
		bias = BIT(4);
	}
	/* program DMIC LDO, clock and set clock */
	snd_soc_update_bits(w->codec, SN95031_MICBIAS, BIT(5)|BIT(4), ldo);
	snd_soc_update_bits(w->codec, SN95031_DMICBUF0123, BIT(2), clk_dir);
	snd_soc_update_bits(w->codec, SN95031_DMICBUF45, BIT(1), data_dir);
	snd_soc_update_bits(w->codec, SN95031_DMICMUX, BIT(4), bias);
	return 0;
}

static int sn95031_dmic56_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	unsigned int ldo = 0, clk_dir = 0, data_dir = 0, bias = 0;

	pr_debug("sn95031_dmic56_event\n");
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("sn95031_dmic56_event ON\n");
		ldo = BIT(7)|BIT(6);
		clk_dir = BIT(4);
		data_dir = BIT(3);
		bias = BIT(5);
	}

	/* program DMIC LDO */
	snd_soc_update_bits(w->codec, SN95031_MICBIAS, BIT(7)|BIT(6), ldo);
	snd_soc_update_bits(w->codec, SN95031_DMICBUF0123, BIT(4), clk_dir);
	snd_soc_update_bits(w->codec, SN95031_DMICBUF45, BIT(3), data_dir);
	snd_soc_update_bits(w->codec, SN95031_DMICMUX, BIT(5), bias);
	return 0;
}

static int sn95031_enable_pnw_clk(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	int clk_id = 0;

	if (!strcmp(w->name, "Vibra1Clock"))
		clk_id = SST_PLL_VIBRA1;
	else if (!strcmp(w->name, "Vibra2Clock"))
		clk_id = SST_PLL_VIBRA2;

	if (SND_SOC_DAPM_EVENT_ON(event))
		intel_sst_set_pll(true, clk_id);
	else if (SND_SOC_DAPM_EVENT_OFF(event))
		intel_sst_set_pll(false, clk_id);
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
	sst_pll_mode_saved = intel_sst_get_pll();
	intel_sst_set_pll(true, SST_PLL_MSIC);
	udelay(SST_PLL_DELAY);
	snd_soc_dapm_force_enable_pin(&codec->dapm, "VirtBias");
	snd_soc_dapm_sync(&codec->dapm);

	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		goto restore_state;

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
restore_state:
	snd_soc_dapm_disable_pin(&codec->dapm, "VirtBias");
	snd_soc_dapm_sync(&codec->dapm);
	if ((sst_pll_mode_saved & SST_PLL_MSIC) == 0)
		intel_sst_set_pll(false, SST_PLL_MSIC);
	mutex_unlock(&codec->mutex);
	return err;
}

/* mux controls */
static const char *sn95031_mic_texts[] = { "AMIC", "LineIn" };

static const struct soc_enum sn95031_micl_enum =
	SOC_ENUM_SINGLE(SN95031_ADCCONFIG, 1, 2, sn95031_mic_texts);

static const struct snd_kcontrol_new sn95031_micl_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_micl_enum);

static const struct soc_enum sn95031_micr_enum =
	SOC_ENUM_SINGLE(SN95031_ADCCONFIG, 3, 2, sn95031_mic_texts);

static const struct snd_kcontrol_new sn95031_micr_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_micr_enum);

static const char *sn95031_input_texts[] = {	"DMIC1", "DMIC2", "DMIC3",
						"DMIC4", "DMIC5", "DMIC6",
						"ADC Left", "ADC Right" };

static const struct soc_enum sn95031_input1_enum =
	SOC_ENUM_SINGLE(SN95031_AUDIOMUX12, 0, 8, sn95031_input_texts);

static const struct snd_kcontrol_new sn95031_input1_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_input1_enum);

static const struct soc_enum sn95031_input2_enum =
	SOC_ENUM_SINGLE(SN95031_AUDIOMUX12, 4, 8, sn95031_input_texts);

static const struct snd_kcontrol_new sn95031_input2_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_input2_enum);

static const struct soc_enum sn95031_input3_enum =
	SOC_ENUM_SINGLE(SN95031_AUDIOMUX34, 0, 8, sn95031_input_texts);

static const struct snd_kcontrol_new sn95031_input3_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_input3_enum);

static const struct soc_enum sn95031_input4_enum =
	SOC_ENUM_SINGLE(SN95031_AUDIOMUX34, 4, 8, sn95031_input_texts);

static const struct snd_kcontrol_new sn95031_input4_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_input4_enum);

/* capture path controls */

static const char *sn95031_micmode_text[] = {"Single Ended", "Differential"};

/* {0, 9, 21, 30} db */
static unsigned int mic_tlv[] = {
	TLV_DB_RANGE_HEAD(4),
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(900, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(2100, 0, 0),
	3, 3, TLV_DB_SCALE_ITEM(3000, 0, 0),
};
/* -62db to 9 db in 1db steps*/
static const DECLARE_TLV_DB_SCALE(out_tlv, -6200, 100, 0);

static const struct soc_enum sn95031_micmode1_enum =
	SOC_ENUM_SINGLE(SN95031_MICAMP1, 1, 2, sn95031_micmode_text);
static const struct soc_enum sn95031_micmode2_enum =
	SOC_ENUM_SINGLE(SN95031_MICAMP2, 1, 2, sn95031_micmode_text);

static const char *sn95031_dmic_cfg_text[] = {"GPO", "DMIC"};

static const struct soc_enum sn95031_dmic12_cfg_enum =
	SOC_ENUM_SINGLE(SN95031_DMICMUX, 0, 2, sn95031_dmic_cfg_text);
static const struct soc_enum sn95031_dmic34_cfg_enum =
	SOC_ENUM_SINGLE(SN95031_DMICMUX, 1, 2, sn95031_dmic_cfg_text);
static const struct soc_enum sn95031_dmic56_cfg_enum =
	SOC_ENUM_SINGLE(SN95031_DMICMUX, 2, 2, sn95031_dmic_cfg_text);

static const char *sn95031_mode_cfg_text[] = {"Music", "Voice"};
static const struct soc_enum sn95031_hsdacsrc_cfg_enum =
	SOC_ENUM_SINGLE(SN95031_HSEPRXCTRL, 6, 2, sn95031_mode_cfg_text);
static const struct snd_kcontrol_new sn95031_hsdacsrc_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_hsdacsrc_cfg_enum);

static const struct soc_enum sn95031_ihfsrc_cfg_enum =
	SOC_ENUM_DOUBLE(SN95031_IHFRXCTRL, 2, 3, 2, sn95031_mode_cfg_text);
static const struct snd_kcontrol_new sn95031_ihfsrc_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_ihfsrc_cfg_enum);
/* TODO: Need to find better way to disable HS while Voice out to IHF */
static const char *sn95031_hsdrv_cfg_text[] = {"Enable", "Disable"};
static const struct soc_enum sn95031_hsdrv_cfg_enum =
	SOC_ENUM_SINGLE(SN95031_DRIVEREN, 7, 2, sn95031_hsdrv_cfg_text);
static const struct snd_kcontrol_new sn95031_hsdrv_mux_control =
	SOC_DAPM_ENUM("Route", sn95031_hsdrv_cfg_enum);

static const char *sn95031_vibra_src_text[] = {"PWM", "SPI"};
static const struct soc_enum sn95031_vibra1_src_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C5, 0, 2, sn95031_vibra_src_text);
static const struct snd_kcontrol_new sn95031_vibra1_src_control =
	SOC_DAPM_ENUM("Route", sn95031_vibra1_src_enum);
static const struct soc_enum sn95031_vibra2_src_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C5, 0, 2, sn95031_vibra_src_text);
static const struct snd_kcontrol_new sn95031_vibra2_src_control =
	SOC_DAPM_ENUM("Route", sn95031_vibra2_src_enum);

static const char *sn95031_vibra_dirn_text[] = {"Forward", "Forward & Reverse"};
static const struct soc_enum sn95031_vibra1_dirn_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C1, 0, 2, sn95031_vibra_dirn_text);
static const struct soc_enum sn95031_vibra2_dirn_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C1, 0, 2, sn95031_vibra_dirn_text);
static const char *sn95031_vibra_boost_text[] = {"0", "1", "2", "4",
						 "8", "16", "32", "64"};
static const struct soc_enum sn95031_vibra1_boost_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C3, 4, 8, sn95031_vibra_boost_text);
static const struct soc_enum sn95031_vibra2_boost_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C3, 4, 8, sn95031_vibra_boost_text);
static const char *sn95031_vibra_cycles_text[] = {"Off", "1", "2", "4",
						 "8", "16", "32", "Infinite"};
static const struct soc_enum sn95031_vibra1_cycles_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C3, 0, 8, sn95031_vibra_cycles_text);
static const struct soc_enum sn95031_vibra2_cycles_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C3, 0, 8, sn95031_vibra_cycles_text);
static const char *sn95031_vibra_duty_text[] = {"20", "22", "24", "26", "28",
						"30", "32", "34", "36", "38",
						"40", "42", "44", "46", "48",
						"50", "52", "54", "56", "58",
						"60", "62", "64", "66", "68",
						"70", "75", "80", "85", "90",
						"95", "100"};
static const struct soc_enum sn95031_vibra1_duty_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C1, 3, 32, sn95031_vibra_duty_text);
static const struct soc_enum sn95031_vibra2_duty_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C1, 3, 32, sn95031_vibra_duty_text);
static const char *sn95031_vibra_on_text[] = {"0.05", "0.10", "0.15", "0.20",
					"0.25", "0.30", "0.35", "0.40",
					"0.45", "0.50", "0.75", "1.00",
					"1.25", "1.5", "2.00", "Infinite"};
static const struct soc_enum sn95031_vibra1_on_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C2, 0, 16, sn95031_vibra_on_text);
static const struct soc_enum sn95031_vibra2_on_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C2, 0, 16, sn95031_vibra_on_text);
static const char *sn95031_vibra_off_text[] = {"0.00", "0.05", "0.10", "0.15",
					"0.20", "0.25", "0.30", "0.35",
					"0.40", "0.45", "0.50", "0.75",
					"1.00", "1.25", "1.50", "2.00"};
static const struct soc_enum sn95031_vibra1_off_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C2, 4, 16, sn95031_vibra_off_text);
static const struct soc_enum sn95031_vibra2_off_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C2, 4, 16, sn95031_vibra_off_text);
static const char *sn95031_vibra_start_text[] = {"Off", "On"};
static const struct soc_enum sn95031_vibra1_start_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C1, 2, 2, sn95031_vibra_start_text);
static const struct soc_enum sn95031_vibra2_start_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C1, 2, 2, sn95031_vibra_start_text);
static const struct soc_enum sn95031_vibra1_brake_enum =
	SOC_ENUM_SINGLE(SN95031_VIB1C1, 1, 2, sn95031_vibra_start_text);
static const struct soc_enum sn95031_vibra2_brake_enum =
	SOC_ENUM_SINGLE(SN95031_VIB2C1, 1, 2, sn95031_vibra_start_text);

static const  char *sn95031_jack_debounce_text[] = {"61us", "122us", "244us",
					"488us", "976us", "1.952ms",
					"3.904ms", "7.8125ms", "15.625ms",
					"31.25ms", "62.5ms", "125ms"};
static const struct soc_enum sn95031_jack_debounce_enum =
	SOC_ENUM_SINGLE(SN95031_BTNCTRL1, 4, 12, sn95031_jack_debounce_text);

static const  char *sn95031_dac_mode_text[] = {"Low Power", "High Performance"};
static const struct soc_enum sn95031_dac_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, sn95031_dac_mode_text);
static unsigned int sn95031_dac_mode;
static int sn95031_set_dac_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 mode = ucontrol->value.integer.value[0];
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (ucontrol->value.integer.value[0] == sn95031_dac_mode)
		return 0;
	if (mode) {
		pr_debug("setting hp mode\n");
		/* aprt from the lp bit we also need to disbale the SSR2
		 * PWRMODE bit.
		 */
		snd_soc_update_bits(codec, SN95031_SSR2, 0x10, 0);
		snd_soc_update_bits(codec, SN95031_DACCONFIG, 0x10, 0x10);
	} else {
		pr_debug("setting lp mode\n");
		snd_soc_update_bits(codec, SN95031_SSR2, 0x10, 0x10);
		snd_soc_update_bits(codec, SN95031_DACCONFIG, 0x10, 0);
	}
	snd_soc_dapm_sync(&codec->dapm);
	sn95031_dac_mode = ucontrol->value.integer.value[0];
	return 0;

}
static int sn95031_get_dac_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = sn95031_dac_mode;
	return 0;
}

static const struct snd_kcontrol_new sn95031_snd_controls[] = {
	SOC_ENUM("Mic1Mode Capture Route", sn95031_micmode1_enum),
	SOC_ENUM("Mic2Mode Capture Route", sn95031_micmode2_enum),
	SOC_ENUM("DMIC12 Capture Route", sn95031_dmic12_cfg_enum),
	SOC_ENUM("DMIC34 Capture Route", sn95031_dmic34_cfg_enum),
	SOC_ENUM("DMIC56 Capture Route", sn95031_dmic56_cfg_enum),
	SOC_SINGLE_TLV("Mic1 Capture Volume", SN95031_MICAMP1,
			2, 3, 0, mic_tlv),
	SOC_SINGLE_TLV("Mic2 Capture Volume", SN95031_MICAMP2,
			2, 3, 0, mic_tlv),
	/* Add digital volume and mute controls for Headphone/Headset*/
	SOC_DOUBLE_R_EXT_TLV("Headphone Playback Volume", SN95031_HSLVOLCTRL,
				SN95031_HSRVOLCTRL, 0, 71, 1,
				snd_soc_get_volsw_2r, sn95031_set_vol_2r,
				out_tlv),
	SOC_DOUBLE_R("Headphone Playback Switch", SN95031_HSLVOLCTRL,
				SN95031_HSRVOLCTRL, 7, 1, 0),
	/* Add digital volume and mute controls for Speaker*/
	SOC_DOUBLE_R_EXT_TLV("Speaker Playback Volume", SN95031_IHFLVOLCTRL,
				SN95031_IHFRVOLCTRL, 0, 71, 1,
				snd_soc_get_volsw_2r, sn95031_set_vol_2r,
				out_tlv),
	SOC_DOUBLE_R("Speaker Playback Switch", SN95031_IHFLVOLCTRL,
				SN95031_IHFRVOLCTRL, 7, 1, 0),

	SOC_ENUM("Vibra1 Direction", sn95031_vibra1_dirn_enum),
	SOC_ENUM("Vibra1 Boost Time", sn95031_vibra1_boost_enum),
	SOC_ENUM("Vibra1 Cycle Count", sn95031_vibra1_cycles_enum),
	SOC_ENUM("Vibra1 Duty Cycle", sn95031_vibra1_duty_enum),
	SOC_ENUM("Vibra1 On Time", sn95031_vibra1_on_enum),
	SOC_ENUM("Vibra1 Off Time", sn95031_vibra1_off_enum),
	SOC_ENUM("Vibra1 Start", sn95031_vibra1_start_enum),
	SOC_ENUM("Vibra1 Brake", sn95031_vibra1_brake_enum),
	SOC_ENUM("Vibra2 Direction", sn95031_vibra2_dirn_enum),
	SOC_ENUM("Vibra2 Boost Time", sn95031_vibra2_boost_enum),
	SOC_ENUM("Vibra2 Cycle Count", sn95031_vibra2_cycles_enum),
	SOC_ENUM("Vibra2 Duty Cycle", sn95031_vibra2_duty_enum),
	SOC_ENUM("Vibra2 On Time", sn95031_vibra2_on_enum),
	SOC_ENUM("Vibra2 Off Time", sn95031_vibra2_off_enum),
	SOC_ENUM("Vibra2 Start", sn95031_vibra2_start_enum),
	SOC_ENUM("Vibra2 Brake", sn95031_vibra2_brake_enum),
	SOC_ENUM("Jack Debounce Time", sn95031_jack_debounce_enum),
	SOC_ENUM_EXT("DAC Mode", sn95031_dac_mode_enum,
			sn95031_get_dac_mode, sn95031_set_dac_mode),
};

/* DAPM widgets */
static const struct snd_soc_dapm_widget sn95031_dapm_widgets[] = {

	/* all end points mic, hs etc */
	SND_SOC_DAPM_OUTPUT("HPOUTL"),
	SND_SOC_DAPM_OUTPUT("HPOUTR"),
	SND_SOC_DAPM_OUTPUT("EPOUT"),
	SND_SOC_DAPM_OUTPUT("IHFOUTL"),
	SND_SOC_DAPM_OUTPUT("IHFOUTR"),
	SND_SOC_DAPM_OUTPUT("LINEOUTL"),
	SND_SOC_DAPM_OUTPUT("LINEOUTR"),
	SND_SOC_DAPM_OUTPUT("VIB1OUT"),
	SND_SOC_DAPM_OUTPUT("VIB2OUT"),

	SND_SOC_DAPM_INPUT("AMIC1"), /* headset mic */
	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),
	SND_SOC_DAPM_INPUT("DMIC3"),
	SND_SOC_DAPM_INPUT("DMIC4"),
	SND_SOC_DAPM_INPUT("DMIC5"),
	SND_SOC_DAPM_INPUT("DMIC6"),
	SND_SOC_DAPM_INPUT("LINEINL"),
	SND_SOC_DAPM_INPUT("LINEINR"),
	SND_SOC_DAPM_INPUT("VIB1SPI"), /* SPI controller as i/p for vibra */
	SND_SOC_DAPM_INPUT("VIB2SPI"),

	SND_SOC_DAPM_MICBIAS("AMIC1Bias", SN95031_MICBIAS, 2, 0),
	SND_SOC_DAPM_MICBIAS("AMIC2Bias", SN95031_MICBIAS, 3, 0),
	SND_SOC_DAPM_MICBIAS("DMIC12Bias", SN95031_DMICMUX, 3, 0),
	SND_SOC_DAPM_MICBIAS("DMIC34Bias", SN95031_DMICMUX, 4, 0),
	SND_SOC_DAPM_MICBIAS("DMIC56Bias", SN95031_DMICMUX, 5, 0),
	/* Dummy widget to trigger VAUDA on/off */
	SND_SOC_DAPM_MICBIAS("VirtBias", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_SUPPLY("DMIC12supply", SN95031_DMICLK, 0, 0,
				sn95031_dmic12_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC34supply", SN95031_DMICLK, 1, 0,
				sn95031_dmic34_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC56supply", SN95031_DMICLK, 2, 0,
				sn95031_dmic56_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT("PCM2_Out", "Capture", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("PCM2_IN", "Headset", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("PCM1_IN", "Downlink", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PCM1_Out", "Uplink", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("Headset Rail", SND_SOC_NOPM, 0, 0,
			sn95031_vhs_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("Speaker Rail", SND_SOC_NOPM, 0, 0,
			sn95031_vihf_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("Vibra1Clock", SND_SOC_NOPM, 0, 0,
			sn95031_enable_pnw_clk,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("Vibra2Clock", SND_SOC_NOPM, 0, 0,
			sn95031_enable_pnw_clk,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* playback path driver enables */
	SND_SOC_DAPM_OUT_DRV("Headset Left Playback",
			SN95031_DRIVEREN, 0, 0, NULL, 0),
	SND_SOC_DAPM_OUT_DRV("Headset Right Playback",
			SN95031_DRIVEREN, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Speaker Left Playback",
			SN95031_DRIVEREN, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Speaker Right Playback",
			SN95031_DRIVEREN, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Vibra1 Playback",
			SN95031_DRIVEREN, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Vibra2 Playback",
			SN95031_DRIVEREN, 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Earpiece Playback",
			SN95031_DRIVEREN, 6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Lineout Left Playback",
			SN95031_LOCTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Lineout Right Playback",
			SN95031_LOCTL, 4, 0, NULL, 0),

	/* playback path filter enable */
	SND_SOC_DAPM_PGA("Headset Left Filter",
			SN95031_HSEPRXCTRL, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Headset Right Filter",
			SN95031_HSEPRXCTRL, 5, 0,  NULL, 0),
	SND_SOC_DAPM_PGA("Speaker Left Filter",
			SN95031_IHFRXCTRL, 0, 0,  NULL, 0),
	SND_SOC_DAPM_PGA("Speaker Right Filter",
			SN95031_IHFRXCTRL, 1, 0,  NULL, 0),

	/* DACs */
	SND_SOC_DAPM_DAC("HSDAC Left", "Headset",
			SN95031_DACCONFIG, 0, 0),
	SND_SOC_DAPM_DAC("HSDAC Right", "Headset",
			SN95031_DACCONFIG, 1, 0),
	SND_SOC_DAPM_DAC("IHFDAC Left", "Speaker",
			SN95031_DACCONFIG, 2, 0),
	SND_SOC_DAPM_DAC("IHFDAC Right", "Speaker",
			SN95031_DACCONFIG, 3, 0),
	SND_SOC_DAPM_DAC("Vibra1 DAC", "Vibra1",
			SN95031_VIB1C5, 1, 0),
	SND_SOC_DAPM_DAC("Vibra2 DAC", "Vibra2",
			SN95031_VIB2C5, 1, 0),
	SND_SOC_DAPM_MUX("Vibra1 Enable Mux",
			SND_SOC_NOPM, 0, 0, &sn95031_vibra1_src_control),
	SND_SOC_DAPM_MUX("Vibra2 Enable Mux",
			SND_SOC_NOPM, 0, 0, &sn95031_vibra2_src_control),

	/* capture widgets */
	SND_SOC_DAPM_PGA("LineIn Enable Left", SN95031_MICAMP1,
				7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LineIn Enable Right", SN95031_MICAMP2,
				7, 0, NULL, 0),

	SND_SOC_DAPM_PGA("MIC1 Enable", SN95031_MICAMP1, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC2 Enable", SN95031_MICAMP2, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("TX1 Enable", SN95031_AUDIOTXEN, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("TX2 Enable", SN95031_AUDIOTXEN, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("TX3 Enable", SN95031_AUDIOTXEN, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("TX4 Enable", SN95031_AUDIOTXEN, 5, 0, NULL, 0),

	/* ADC have null stream as they will be turned ON by TX path */
	SND_SOC_DAPM_ADC("ADC Left", NULL,
			SN95031_ADCCONFIG, 0, 0),
	SND_SOC_DAPM_ADC("ADC Right", NULL,
			SN95031_ADCCONFIG, 2, 0),

	SND_SOC_DAPM_MUX("Mic_InputL Capture Route",
			SND_SOC_NOPM, 0, 0, &sn95031_micl_mux_control),
	SND_SOC_DAPM_MUX("Mic_InputR Capture Route",
			SND_SOC_NOPM, 0, 0, &sn95031_micr_mux_control),

	SND_SOC_DAPM_MUX("Txpath1 Capture Route",
			SND_SOC_NOPM, 0, 0, &sn95031_input1_mux_control),
	SND_SOC_DAPM_MUX("Txpath2 Capture Route",
			SND_SOC_NOPM, 0, 0, &sn95031_input2_mux_control),
	SND_SOC_DAPM_MUX("Txpath3 Capture Route",
			SND_SOC_NOPM, 0, 0, &sn95031_input3_mux_control),
	SND_SOC_DAPM_MUX("Txpath4 Capture Route",
			SND_SOC_NOPM, 0, 0, &sn95031_input4_mux_control),
	SND_SOC_DAPM_MUX("Mode Playback Route",
			SND_SOC_NOPM, 0, 0, &sn95031_hsdacsrc_mux_control),
	SND_SOC_DAPM_MUX("Speaker Mux Playback Route",
			SND_SOC_NOPM, 0, 0, &sn95031_ihfsrc_mux_control),
	SND_SOC_DAPM_MUX("Headset Playback Route",
			SND_SOC_NOPM, 0, 0, &sn95031_hsdrv_mux_control),
};

static const struct snd_soc_dapm_route sn95031_audio_map[] = {
	/* headset and earpiece map */
	{ "HPOUTL", NULL, "Headset Rail"},
	{ "HPOUTR", NULL, "Headset Rail"},
	{ "HPOUTL", NULL, "Headset Left Playback" },
	{ "HPOUTR", NULL, "Headset Right Playback" },
	{ "EPOUT", NULL, "Earpiece Playback" },

	{ "Headset Left Playback", NULL, "Headset Playback Route"},
	{ "Headset Right Playback", NULL, "Headset Playback Route"},
	{ "Headset Playback Route", "Enable", "Headset Left Filter"},
	{ "Headset Playback Route", "Enable", "Headset Right Filter"},

	{ "Earpiece Playback", NULL, "Headset Left Filter"},
	{ "Headset Left Filter", NULL, "HSDAC Left"},
	{ "Headset Right Filter", NULL, "HSDAC Right"},
	{ "HSDAC Left", NULL, "Mode Playback Route"},
	{ "HSDAC Right", NULL, "Mode Playback Route"},
	{ "Mode Playback Route", "Music", "PCM2_IN"},

	/* Voice Playback path*/
	{ "Mode Playback Route", "Voice", "PCM1_IN"},
	/* speaker map */
	{ "IHFOUTL", "NULL", "Speaker Left Playback"},
	{ "IHFOUTR", "NULL", "Speaker Right Playback"},
	{ "Speaker Left Playback", "Music", "Speaker Mux Playback Route"},
	{ "Speaker Right Playback", "Music", "Speaker Mux Playback Route"},
	{ "Speaker Mux Playback Route", "Music", "Speaker Left Filter"},
	{ "Speaker Mux Playback Route", "Music", "Speaker Right Filter"},
	{ "Speaker Mux Playback Route", "Voice", "Headset Left Filter"},
	{ "Speaker Mux Playback Route", "Voice", "Headset Right Filter"},
	{ "Speaker Left Filter", NULL, "IHFDAC Left"},
	{ "Speaker Right Filter", NULL, "IHFDAC Right"},

	/* vibra map */
	{ "VIB1OUT", NULL, "Vibra1 Playback"},
	{ "Vibra1 Playback", NULL, "Vibra1 Enable Mux"},
	{ "Vibra1 Enable Mux", "PWM", "Vibra1 DAC"},
	{ "Vibra1 Enable Mux", "SPI", "VIB1SPI"},
	{ "VIB1SPI", NULL, "Vibra1Clock"},

	{ "VIB2OUT", NULL, "Vibra2 Playback"},
	{ "Vibra2 Playback", NULL, "Vibra2 Enable Mux"},
	{ "Vibra2 Enable Mux", "PWM", "Vibra2 DAC"},
	{ "Vibra2 Enable Mux", "SPI", "VIB2SPI"},
	{ "VIB2SPI", NULL, "Vibra2Clock"},

	/* lineout */
	{ "LINEOUTL", NULL, "Lineout Left Playback"},
	{ "LINEOUTR", NULL, "Lineout Right Playback"},
	{ "Lineout Left Playback", NULL, "Headset Left Filter"},
	{ "Lineout Left Playback", NULL, "Speaker Left Filter"},
	{ "Lineout Left Playback", NULL, "Vibra1 DAC"},
	{ "Lineout Right Playback", NULL, "Headset Right Filter"},
	{ "Lineout Right Playback", NULL, "Speaker Right Filter"},
	{ "Lineout Right Playback", NULL, "Vibra2 DAC"},

	/* Headset (AMIC1) mic */
	{ "AMIC1Bias", NULL, "AMIC1"},
	{ "MIC1 Enable", NULL, "AMIC1Bias"},
	{ "Mic_InputL Capture Route", "AMIC", "MIC1 Enable"},

	/* AMIC2 */
	{ "AMIC2Bias", NULL, "AMIC2"},
	{ "MIC2 Enable", NULL, "AMIC2Bias"},
	{ "Mic_InputR Capture Route", "AMIC", "MIC2 Enable"},


	/* Linein */
	{ "LineIn Enable Left", NULL, "LINEINL"},
	{ "LineIn Enable Right", NULL, "LINEINR"},
	{ "Mic_InputL Capture Route", "LineIn", "LineIn Enable Left"},
	{ "Mic_InputR Capture Route", "LineIn", "LineIn Enable Right"},

	/* ADC connection */
	{ "ADC Left", NULL, "Mic_InputL Capture Route"},
	{ "ADC Right", NULL, "Mic_InputR Capture Route"},

	/*DMIC connections */
	{ "DMIC1", NULL, "DMIC12supply"},
	{ "DMIC2", NULL, "DMIC12supply"},
	{ "DMIC3", NULL, "DMIC34supply"},
	{ "DMIC4", NULL, "DMIC34supply"},
	{ "DMIC5", NULL, "DMIC56supply"},
	{ "DMIC6", NULL, "DMIC56supply"},

	{ "DMIC12Bias", NULL, "DMIC1"},
	{ "DMIC12Bias", NULL, "DMIC2"},
	{ "DMIC34Bias", NULL, "DMIC3"},
	{ "DMIC34Bias", NULL, "DMIC4"},
	{ "DMIC56Bias", NULL, "DMIC5"},
	{ "DMIC56Bias", NULL, "DMIC6"},

	/*TX path inputs*/
	{ "Txpath1 Capture Route", "ADC Left", "ADC Left"},
	{ "Txpath2 Capture Route", "ADC Left", "ADC Left"},
	{ "Txpath3 Capture Route", "ADC Left", "ADC Left"},
	{ "Txpath4 Capture Route", "ADC Left", "ADC Left"},
	{ "Txpath1 Capture Route", "ADC Right", "ADC Right"},
	{ "Txpath2 Capture Route", "ADC Right", "ADC Right"},
	{ "Txpath3 Capture Route", "ADC Right", "ADC Right"},
	{ "Txpath4 Capture Route", "ADC Right", "ADC Right"},
	{ "Txpath1 Capture Route", "DMIC1", "DMIC1"},
	{ "Txpath2 Capture Route", "DMIC1", "DMIC1"},
	{ "Txpath3 Capture Route", "DMIC1", "DMIC1"},
	{ "Txpath4 Capture Route", "DMIC1", "DMIC1"},
	{ "Txpath1 Capture Route", "DMIC2", "DMIC2"},
	{ "Txpath2 Capture Route", "DMIC2", "DMIC2"},
	{ "Txpath3 Capture Route", "DMIC2", "DMIC2"},
	{ "Txpath4 Capture Route", "DMIC2", "DMIC2"},
	{ "Txpath1 Capture Route", "DMIC3", "DMIC3"},
	{ "Txpath2 Capture Route", "DMIC3", "DMIC3"},
	{ "Txpath3 Capture Route", "DMIC3", "DMIC3"},
	{ "Txpath4 Capture Route", "DMIC3", "DMIC3"},
	{ "Txpath1 Capture Route", "DMIC4", "DMIC4"},
	{ "Txpath2 Capture Route", "DMIC4", "DMIC4"},
	{ "Txpath3 Capture Route", "DMIC4", "DMIC4"},
	{ "Txpath4 Capture Route", "DMIC4", "DMIC4"},
	{ "Txpath1 Capture Route", "DMIC5", "DMIC5"},
	{ "Txpath2 Capture Route", "DMIC5", "DMIC5"},
	{ "Txpath3 Capture Route", "DMIC5", "DMIC5"},
	{ "Txpath4 Capture Route", "DMIC5", "DMIC5"},
	{ "Txpath1 Capture Route", "DMIC6", "DMIC6"},
	{ "Txpath2 Capture Route", "DMIC6", "DMIC6"},
	{ "Txpath3 Capture Route", "DMIC6", "DMIC6"},
	{ "Txpath4 Capture Route", "DMIC6", "DMIC6"},

	/* PCM2 Tx path */
	{ "TX1 Enable", NULL, "Txpath1 Capture Route"},
	{ "TX2 Enable", NULL, "Txpath2 Capture Route"},
	{ "TX3 Enable", NULL, "Txpath3 Capture Route"},
	{ "TX4 Enable", NULL, "Txpath4 Capture Route"},
	{ "PCM2_Out", NULL, "TX1 Enable"},
	{ "PCM2_Out", NULL, "TX2 Enable"},
	{ "PCM2_Out", NULL, "TX3 Enable"},
	{ "PCM2_Out", NULL, "TX4 Enable"},
	/* PCM1 Tx path */
	{ "PCM1_Out", NULL, "TX1 Enable"},
	{ "PCM1_Out", NULL, "TX2 Enable"},
	{ "PCM1_Out", NULL, "TX3 Enable"},
	{ "PCM1_Out", NULL, "TX4 Enable"},
};

/* speaker and headset mutes, for audio pops and clicks */
static int sn95031_pcm_hs_mute(struct snd_soc_dai *dai, int mute)
{
	snd_soc_update_bits(dai->codec,
			SN95031_HSLVOLCTRL, BIT(7), (!mute << 7));
	snd_soc_update_bits(dai->codec,
			SN95031_HSRVOLCTRL, BIT(7), (!mute << 7));
	return 0;
}

static int sn95031_pcm_spkr_mute(struct snd_soc_dai *dai, int mute)
{
	snd_soc_update_bits(dai->codec,
			SN95031_IHFLVOLCTRL, BIT(7), (!mute << 7));
	snd_soc_update_bits(dai->codec,
			SN95031_IHFRVOLCTRL, BIT(7), (!mute << 7));
	return 0;
}

static int sn95031_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	int mode, target_clk_src;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec_dai->codec);

	pr_debug("set_dai_pll\n");

	mutex_lock(&codec->mutex);
	if (!freq_in || !freq_out) {
		/* disable PLL  */
		sn95031_configure_pll(codec, DISABLE_PLL);
		mutex_unlock(&codec->mutex);
		return 0;
	}
	mode = snd_soc_read(codec, SN95031_PCM1C3) >> 7;
	if (!mode && (!strcmp(codec_dai->name, "SN95031 Voice"))) {
		target_clk_src = SN95031_PCM1BCLK;
		snd_soc_write(codec, SN95031_PCM1C2, 0x04);
	} else {
		target_clk_src = SN95031_PLLIN;
		snd_soc_write(codec, SN95031_PCM1C2, 0x00);
	}
	/* clock source is same, so don't do anything */
	if (sn95031_ctx->clk_src == target_clk_src) {
		pr_debug("clk src is same, no action\n");
		mutex_unlock(&codec->mutex);
		return 0;
	}
	sn95031_ctx->clk_src = target_clk_src;
	if (codec->dapm.bias_level >= SND_SOC_BIAS_PREPARE) {
		pr_debug("bias_level is active, enabling pll\n");
		intel_sst_set_pll(true, SST_PLL_MSIC);
		/* allow few ms to stabilize the clock before
			enabling the MSIC PLL */
		usleep_range(5000, 6000);
		sn95031_configure_pll(codec, ENABLE_PLL);
	} else
		sn95031_ctx->pll_state = PLL_ENABLE_PENDING;

	mutex_unlock(&codec->mutex);
	return 0;
}

static int sn95031_set_pcm2_tristate(struct snd_soc_dai *codec_dai,
							int tristate)
{
	u8 val;

	pr_debug("enter:%s\n", __func__);
	if (tristate)
		val = 0;
	else
		val = 1;

	return snd_soc_update_bits(codec_dai->codec, SN95031_PCM2C2,
						BIT(0), val);
}

static int sn95031_codec_set_params(struct snd_soc_codec *codec,
						unsigned int param)
{
	unsigned int format;

	pr_debug("enter:%s\n", __func__);
	switch (param) {
	case SNDRV_PCM_FORMAT_S16_LE:
		format = BIT(4)|BIT(5);
		break;

	case SNDRV_PCM_FORMAT_S24_LE:
		format = 0;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_update_bits(codec, SN95031_PCM2C2,
			BIT(4)|BIT(5), format);
	/* enable pcm 2 */
	snd_soc_update_bits(codec, SN95031_PCM2C2, BIT(0), BIT(0));
	return 0;
}

static int sn95031_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	unsigned int rate;

	pr_debug("pcm_hw param\n");
	switch (params_rate(params)) {
	case 48000:
		pr_debug("RATE_48000\n");
		rate = 0;
		break;

	case 44100:
		pr_debug("RATE_44100\n");
		rate = BIT(7);
		break;

	default:
		pr_err("ERR rate %d\n", params_rate(params));
		return -EINVAL;
	}
	snd_soc_update_bits(dai->codec, SN95031_PCM1C1, BIT(7), rate);

	return 0;
}

static int sn95031_voice_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	unsigned int format, pcm1fs, rate = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		format = BIT(4)|BIT(5);
		break;

	case SNDRV_PCM_FORMAT_S24_LE:
		format = 0;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_update_bits(dai->codec, SN95031_PCM1C3,
			BIT(4)|BIT(5), format);

	switch (params_rate(params)) {
	case 8000:
		pcm1fs = 0;
		break;
	case 12000:
		pcm1fs = BIT(4);
		break;
	case 16000:
		pcm1fs = BIT(5);
		break;
	case 24000:
		pcm1fs = BIT(5)|BIT(4);
		break;
	case 48000:
		pcm1fs = BIT(6)|BIT(4);
		break;
	case 96000:
		pcm1fs = BIT(6)|BIT(5)|BIT(4);
		break;
	case 11025:
		pcm1fs = BIT(4);
		rate = BIT(7);
		break;
	case 22050:
		pcm1fs = BIT(5)|BIT(4);
		rate = BIT(7);
		break;
	case 44100:
		pcm1fs = BIT(6)|BIT(4);
		rate = BIT(7);
		break;
	case 88200:
		pcm1fs = BIT(6)|BIT(5)|BIT(4);
		rate = BIT(7);
		break;
	default:
		pr_err("ERR rate %d\n", params_rate(params));
		return -EINVAL;
	}
	pr_debug("rate=%d\n", params_rate(params));
	snd_soc_update_bits(dai->codec, SN95031_PCM1C1, BIT(7), rate);
	snd_soc_update_bits(dai->codec, SN95031_PCM1C1, BIT(6)|BIT(5)|BIT(4),
						pcm1fs);
	/* enable pcm 1 */
	snd_soc_update_bits(dai->codec, SN95031_PCM1C3, BIT(0), BIT(0));
	return 0;
}

static int sn95031_voice_hw_free(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	pr_debug("inside hw_free");
	snd_soc_update_bits(dai->codec, SN95031_PCM1C3, BIT(0), 0);
	/* PCM1 should be in slave, short or long sync mode for
		Tx line to be in Hi-Z state */
	snd_soc_update_bits(dai->codec, SN95031_PCM1C3, BIT(7), 0);
	snd_soc_write(dai->codec, SN95031_PCM1C2, 0x00);
	return 0;
}

/* Codec DAI section */
static struct snd_soc_dai_ops sn95031_headset_dai_ops = {
	.digital_mute	= sn95031_pcm_hs_mute,
	.hw_params	= sn95031_pcm_hw_params,
	.set_pll	= sn95031_set_dai_pll,
	.set_tristate	= sn95031_set_pcm2_tristate,
};

static struct snd_soc_dai_ops sn95031_speaker_dai_ops = {
	.digital_mute	= sn95031_pcm_spkr_mute,
	.hw_params	= sn95031_pcm_hw_params,
	.set_pll	= sn95031_set_dai_pll,
	.set_tristate	= sn95031_set_pcm2_tristate,
};

static struct snd_soc_dai_ops sn95031_vib1_dai_ops = {
	.hw_params	= sn95031_pcm_hw_params,
	.set_pll	= sn95031_set_dai_pll,
	.set_tristate	= sn95031_set_pcm2_tristate,
};

static struct snd_soc_dai_ops sn95031_vib2_dai_ops = {
	.hw_params	= sn95031_pcm_hw_params,
	.set_pll	= sn95031_set_dai_pll,
	.set_tristate	= sn95031_set_pcm2_tristate,
};

static struct snd_soc_dai_ops sn95031_voice_dai_ops = {
	.digital_mute	= sn95031_pcm_hs_mute,
	.hw_params	= sn95031_voice_hw_params,
	.hw_free	= sn95031_voice_hw_free,
	.set_pll	= sn95031_set_dai_pll,
};

static struct snd_soc_dai_driver sn95031_dais[] = {
{
	.name = "SN95031 Headset",
	.playback = {
		.stream_name = "Headset",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 5,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.ops = &sn95031_headset_dai_ops,
},
{	.name = "SN95031 Speaker",
	.playback = {
		.stream_name = "Speaker",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.ops = &sn95031_speaker_dai_ops,
},
{	.name = "SN95031 Vibra1",
	.playback = {
		.stream_name = "Vibra1",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.ops = &sn95031_vib1_dai_ops,
},
{	.name = "SN95031 Vibra2",
	.playback = {
		.stream_name = "Vibra2",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.ops = &sn95031_vib2_dai_ops,
},
{
	.name = "SN95031 Voice",
	.playback = {
		.stream_name = "Downlink",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.capture = {
		.stream_name = "Uplink",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.ops = &sn95031_voice_dai_ops,
},
};

static inline void sn95031_disable_jack_btn(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, SN95031_BTNCTRL2, BIT(0), 0);
}

static inline void sn95031_enable_jack_btn(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, SN95031_BTNCTRL2, BIT(0), BIT(0));
}

static int sn95031_get_headset_state(struct snd_soc_jack *mfld_jack)
{
	int micbias, jack_type, hs_gpio = 1;

	sn95031_enable_mic_bias(mfld_jack->codec);
	micbias = sn95031_read_voltage();

	jack_type = snd_soc_jack_get_type(mfld_jack, micbias);
	pr_debug("jack type detected = %d, micbias = %d\n", jack_type, micbias);

	if (mfld_board_id() == MFLD_BID_PR3) {
		if ((jack_type != SND_JACK_HEADSET) &&
		    (jack_type != SND_JACK_HEADPHONE))
			hs_gpio = gpio_get_value(HEADSET_DET_PIN);
		if (!hs_gpio) {
			jack_type = SND_JACK_HEADPHONE;
			pr_debug("GPIO says there is a headphone, reporting it\n");
		}
	}
	if (jack_type == SND_JACK_HEADSET)
		sn95031_enable_jack_btn(mfld_jack->codec);
	else
		sn95031_disable_mic_bias(mfld_jack->codec);

	return jack_type;
}
static void sn95031_jack_report(struct snd_soc_jack *jack, unsigned int status)
{
	unsigned int mask = SND_JACK_BTN_0 | SND_JACK_HEADSET;

	pr_debug("jack reported of type: 0x%x\n", status);
	if ((status == SND_JACK_HEADSET) || (status == SND_JACK_HEADPHONE)) {
		/* if we detected valid headset then disable headset ground.
		 * Otherwise enable it in else condition
		 * this is required for jack detect to work well */
		snd_soc_update_bits(jack->codec, SN95031_BTNCTRL2, BIT(1), 0);
	} else if (status == 0) {
		snd_soc_update_bits(jack->codec,
					SN95031_BTNCTRL2, BIT(1), BIT(1));
	}
	snd_soc_jack_report(jack, status, mask);
#ifdef CONFIG_SWITCH_MID
	/* report to the switch driver as well */
	if (status) {
		if (status == SND_JACK_HEADPHONE)
			mid_headset_report((1<<1));
		else if (status == SND_JACK_HEADSET)
			mid_headset_report(1);
	} else {
		mid_headset_report(0);
	}
#endif
}

void sn95031_jack_wq(struct work_struct *work)
{
	unsigned int mask = SND_JACK_BTN_0 | SND_JACK_HEADSET;
	struct sn95031_priv *sn95031_ctx =
		container_of(work, struct sn95031_priv, jack_work.work.work);
	struct sn95031_jack_work *jack_wq = &sn95031_ctx->jack_work;
	struct snd_soc_jack *jack = jack_wq->jack;
	unsigned int voltage, status = 0;

	pr_debug("jack status in wq: 0x%x\n", jack_wq->intr_id);
	if (jack_wq->intr_id & SN95031_JACK_INSERTED) {
		status = sn95031_get_headset_state(jack);
		jack_wq->intr_id &= ~SN95031_JACK_INSERTED;
		/* unmask button press interrupts */
		if (status == SND_JACK_HEADSET)
			snd_soc_update_bits(jack->codec, SN95031_ACCDETMASK,
							BIT(1)|BIT(0), 0);
		cancel_delayed_work(&sn95031_ctx->jack_work.work);
	} else if (jack_wq->intr_id & SN95031_JACK_REMOVED) {
		if (mfld_board_id() == MFLD_BID_PR3) {
			if (!gpio_get_value(HEADSET_DET_PIN)) {
				pr_debug("remove interrupt, but GPIO says inserted\n");
				return;
			}
		}
		pr_debug("reporting jack as removed\n");
		sn95031_disable_jack_btn(jack->codec);
		snd_soc_update_bits(jack->codec, SN95031_ACCDETMASK, BIT(2), 0);
		sn95031_disable_mic_bias(jack->codec);
		jack_wq->intr_id = 0;
		cancel_delayed_work(&sn95031_ctx->jack_work.work);
	} else if (jack_wq->intr_id & SN95031_JACK_BTN0) {
		jack_wq->intr_id &= ~SN95031_JACK_BTN0;
		if (sn95031_lp_flag) {
			snd_soc_jack_report(jack, SND_JACK_HEADSET, mask);
			sn95031_lp_flag = 0;
			/* clear up BTN1 intr_id if it was not cleared */
			jack_wq->intr_id &= ~SN95031_JACK_BTN1;
			pr_debug("short press intr on releasing long press, "
				   "report button release\n");
			return;
		} else {
			status = SND_JACK_HEADSET | SND_JACK_BTN_0;
			pr_debug("short press detected\n");
			snd_soc_jack_report(jack, status, mask);
			/* send explicit button release */
			if (status & SND_JACK_BTN_0)
				snd_soc_jack_report(jack,
						SND_JACK_HEADSET, mask);
			return;
		}
	} else if (jack_wq->intr_id & SN95031_JACK_BTN1) {
		/* we get spurious interrupts if jack key is held down
		* so we ignore them until key is released by checking the
		* voltage level */
		if (sn95031_lp_flag) {
			voltage = sn95031_read_voltage();
			if (voltage > LP_THRESHOLD) {
				snd_soc_jack_report(jack,
						SND_JACK_HEADSET, mask);
				sn95031_lp_flag = 0;
				jack_wq->intr_id &= ~SN95031_JACK_BTN1;
				pr_debug("button released after long press\n");
			}
			return;
		}
		/* Codec sends separate long press event after button pressed
		 * for a specified time. Need to send separate button pressed
		 * and released events for Android */
		status = SND_JACK_HEADSET | SND_JACK_BTN_0;
		sn95031_lp_flag = 1;
		jack_wq->intr_id &= ~SN95031_JACK_BTN1;
		pr_debug("long press detected\n");
	}
	sn95031_jack_report(jack, status);
}

static int sn95031_schedule_jack_wq(struct mfld_jack_data *jack_data)
{
	int retval = 0;
	struct sn95031_priv *sn95031 = snd_soc_codec_get_drvdata(
			jack_data->mfld_jack->codec);

	sn95031->jack_work.jack = jack_data->mfld_jack;
	retval = schedule_delayed_work(&sn95031->jack_work.work,
			msecs_to_jiffies(SN95031_SW_DBNC));
	return retval;
}

void sn95031_jack_detection(struct mfld_jack_data *jack_data)
{
	int retval = 0;
	struct sn95031_priv *sn95031 = snd_soc_codec_get_drvdata(
			jack_data->mfld_jack->codec);

	pr_debug("interrupt id read in sram = 0x%x\n", jack_data->intr_id);

	if (jack_data->intr_id & SN95031_JACK_INSERTED ||
				jack_data->intr_id & SN95031_JACK_REMOVED) {

		retval = sn95031_schedule_jack_wq(jack_data);
		if (!retval) {
			pr_debug("jack inserted/removed, intr already queued \n");
			sn95031->jack_work.intr_id = jack_data->intr_id;
		} else {
			sn95031->jack_work.intr_id |= jack_data->intr_id;
		}
		/* mask button press interrupts until jack is reported*/
		snd_soc_update_bits(jack_data->mfld_jack->codec,
		     SN95031_ACCDETMASK, BIT(1)|BIT(0), BIT(1)|BIT(0));
		return;
	}

	if (jack_data->intr_id & SN95031_JACK_BTN0 ||
				jack_data->intr_id & SN95031_JACK_BTN1) {
		if ((jack_data->mfld_jack->status & SND_JACK_HEADSET) != 0) {
			retval = sn95031_schedule_jack_wq(jack_data);
			if (!retval) {
				pr_debug("spurious btn press, lp_flag:%d\n",
							sn95031_lp_flag);
				sn95031->jack_work.intr_id = jack_data->intr_id;
				return;
			}
			sn95031->jack_work.intr_id |= jack_data->intr_id;
			pr_debug("BTN_Press detected\n");
		} else {
			pr_debug("BTN_press received, but jack is removed\n");
		}
	}
}
EXPORT_SYMBOL_GPL(sn95031_jack_detection);

/* codec registration */
static int sn95031_codec_probe(struct snd_soc_codec *codec)
{
	struct sn95031_priv *sn95031_ctx;

	pr_debug("codec_probe called\n");

	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	codec->dapm.idle_bias_off = 1;

	sn95031_ctx = kzalloc(sizeof(struct sn95031_priv), GFP_ATOMIC);
	if (!sn95031_ctx) {
		pr_err("codec ctx aloc failed\n");
		return -ENOMEM;
	}
	sn95031_ctx->clk_src = SN95031_INVALID;
	sn95031_ctx->pll_state = PLL_DISABLED;

	INIT_DELAYED_WORK(&sn95031_ctx->jack_work.work, sn95031_jack_wq);

	/* PCM1 slot configurations*/
	snd_soc_write(codec, SN95031_NOISEMUX, 0x0);
	snd_soc_write(codec, SN95031_PCM1RXSLOT0_3, 0xF9);
	snd_soc_write(codec, SN95031_PCM1RXSLOT45, 0x0F);
	snd_soc_write(codec, SN95031_PCM1TXSLOT01, 0x32);
	snd_soc_write(codec, SN95031_PCM1TXSLOT23, 0x77);
	snd_soc_write(codec, SN95031_PCM1TXSLOT45, 0x77);

	/* PCM interface config
	 * This sets the pcm rx slot conguration to max 6 slots
	 * for max 4 dais (2 stereo and 2 mono)
	 */
	snd_soc_write(codec, SN95031_PCM2RXSLOT01, 0x10);
	snd_soc_write(codec, SN95031_PCM2RXSLOT23, 0x32);
	snd_soc_write(codec, SN95031_PCM2RXSLOT45, 0x54);
	snd_soc_write(codec, SN95031_PCM2TXSLOT01, 0x10);
	snd_soc_write(codec, SN95031_PCM2TXSLOT23, 0x32);
	/* pcm port setting
	 * This sets the pcm port to slave and clock at 19.2Mhz which
	 * can support 6slots, sampling rate set per stream in hw-params
	 */
	snd_soc_write(codec, SN95031_PCM1C1, 0x00);
	/* configure pcm1 port in short sync mode */
	snd_soc_write(codec, SN95031_PCM1C2, 0x00);
	snd_soc_write(codec, SN95031_PCM1C3, 0x02);
	snd_soc_write(codec, SN95031_PCM2C1, 0x01);
	snd_soc_write(codec, SN95031_PCM2C2, 0x0A);
	snd_soc_write(codec, SN95031_HSMIXER, BIT(0)|BIT(4));
	/* vendor vibra workround, the vibras are muted by
	 * custom register so unmute them
	 */
	snd_soc_write(codec, SN95031_SSR5, 0x80);
	snd_soc_write(codec, SN95031_SSR6, 0x80);
	snd_soc_write(codec, SN95031_VIB1C5, 0x00);
	snd_soc_write(codec, SN95031_VIB2C5, 0x00);
	/* configure vibras for pcm port */
	snd_soc_write(codec, SN95031_VIB1C3, 0x00);
	snd_soc_write(codec, SN95031_VIB2C3, 0x00);

	snd_soc_write(codec, SN95031_AUDIOMUX12, 0x10);
	snd_soc_write(codec, SN95031_AUDIOMUX34, 0x32);
	/* voice related stuff */
	snd_soc_write(codec, SN95031_VOICETXVOL, 0x89);
	/* debounce time and long press duration */
	snd_soc_write(codec, SN95031_BTNCTRL1, 0x51);

	/* soft mute ramp time */
	snd_soc_write(codec, SN95031_SOFTMUTE, 0x3);
	/* fix the initial volume at -2dB,
	 * default in +9dB,
	 * Gain more than -2dB causes clipping when
	 * VHS is set to  1.2v.
	 */
	snd_soc_write(codec, SN95031_HSLVOLCTRL, 0x0B);
	snd_soc_write(codec, SN95031_HSRVOLCTRL, 0x0B);
	snd_soc_write(codec, SN95031_IHFLVOLCTRL, 0x0B);
	snd_soc_write(codec, SN95031_IHFRVOLCTRL, 0x0B);
	/* dac mode and lineout workaround */
	snd_soc_write(codec, SN95031_SSR2, 0x10);
	snd_soc_write(codec, SN95031_SSR3, 0x40);
	sn95031_dac_mode = 0;

	/* turn off all rails, will be enabled when required */
	snd_soc_write(codec, SN95031_VAUD, 0x24);
	snd_soc_write(codec, SN95031_VIHF, 0x24);
	snd_soc_write(codec, SN95031_VHSN, 0x24);
	snd_soc_write(codec, SN95031_VHSP, 0x24);

	/* mask the OCVOLSTSMASK bit, so that driver will not receive
	 * any overcurrent interrupt. Overcurent scenario will be
	 * handled from the application space through alsa_amixer
	 * volume control.
	 */
	snd_soc_update_bits(codec, SN95031_OCAUDIOMASK, BIT(0), BIT(0));

	snd_soc_add_controls(codec, sn95031_snd_controls,
			     ARRAY_SIZE(sn95031_snd_controls));

	/*GPADC handle for audio_detection*/
	audio_adc_handle = intel_mid_gpadc_alloc(SN95031_AUDIO_SENSOR,
				SN95031_AUDIO_DETECT_CODE);
	if (!audio_adc_handle) {
		pr_err("invalid ADC handle\n");
		kfree(sn95031_ctx);
		return -ENOMEM;
	}

	snd_soc_codec_set_drvdata(codec, sn95031_ctx);

	return 0;
}

static int sn95031_codec_remove(struct snd_soc_codec *codec)
{
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec);

	pr_debug("codec_remove called\n");
	sn95031_set_vaud_bias(codec, SND_SOC_BIAS_OFF);

	/*Free the adc handle*/
	intel_mid_gpadc_free(audio_adc_handle);
	cancel_delayed_work(&sn95031_ctx->jack_work.work);
	kfree(sn95031_ctx);

	return 0;
}
struct snd_soc_codec_driver sn95031_codec = {
	.probe		= sn95031_codec_probe,
	.remove		= sn95031_codec_remove,
	.read		= sn95031_read,
	.write		= sn95031_write,
	.set_bias_level	= sn95031_set_vaud_bias,
	.set_params	= sn95031_codec_set_params,
	.dapm_widgets	= sn95031_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(sn95031_dapm_widgets),
	.dapm_routes	= sn95031_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(sn95031_audio_map),
	.stream_event	= sn95031_codec_stream_event,
};

static int __devinit sn95031_device_probe(struct platform_device *pdev)
{
	pr_debug("codec device probe called for %s\n", dev_name(&pdev->dev));
	return snd_soc_register_codec(&pdev->dev, &sn95031_codec,
			sn95031_dais, ARRAY_SIZE(sn95031_dais));
}

static int __devexit sn95031_device_remove(struct platform_device *pdev)
{
	pr_debug("codec device remove called\n");
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver sn95031_codec_driver = {
	.driver		= {
		.name		= "sn95031",
		.owner		= THIS_MODULE,
	},
	.probe		= sn95031_device_probe,
	.remove		= __devexit_p(sn95031_device_remove),
};

static int __init sn95031_init(void)
{
	pr_debug("driver init called\n");
	return platform_driver_register(&sn95031_codec_driver);
}
module_init(sn95031_init);

static void __exit sn95031_exit(void)
{
	pr_debug("driver exit called\n");
	platform_driver_unregister(&sn95031_codec_driver);
}
module_exit(sn95031_exit);

MODULE_DESCRIPTION("ASoC TI SN95031 codec driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sn95031");
