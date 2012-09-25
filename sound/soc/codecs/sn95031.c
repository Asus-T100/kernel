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
#include <linux/module.h>
#include <linux/gpio.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel-mid.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include "sn95031.h"

#define SN95031_RATES (SNDRV_PCM_RATE_8000_96000)
#define SN95031_FORMATS (SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE)

/* codec private data */
struct sn95031_priv {
	uint8_t clk_src;
	enum sn95031_pll_status pll_state;
};

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

void sn95031_configure_pll(struct snd_soc_codec *codec, int operation)
{
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec);


	if (sn95031_ctx->pll_state == PLL_ENABLE_PENDING
			&& operation == SN95031_ENABLE_PLL) {
		pr_debug("setting PLL to 0x%x\n", sn95031_ctx->clk_src);
		/* PLL takes few msec to stabilize
		Refer sec2.3 MFLD Audio Interface Doc-rev0.7 */
		snd_soc_write(codec, SN95031_AUDPLLCTRL, 0);
		udelay(1000);
		snd_soc_write(codec, SN95031_AUDPLLCTRL,
					(sn95031_ctx->clk_src)<<2);
		udelay(1000);
		snd_soc_update_bits(codec, SN95031_AUDPLLCTRL, BIT(1), BIT(1));
		udelay(1000);
		snd_soc_update_bits(codec, SN95031_AUDPLLCTRL, BIT(5), BIT(5));
		udelay(1000);
		sn95031_ctx->pll_state = PLL_ENABLED;
	} else if (operation == SN95031_DISABLE_PLL) {
		pr_debug("disabling PLL\n");
		snd_soc_write(codec, SN95031_AUDPLLCTRL, 0);
		sn95031_ctx->clk_src = SN95031_INVALID;
		sn95031_ctx->pll_state = PLL_DISABLED;
	} else {
		pr_debug("PLL configure state: op=0x%x, state=0x%x\n",
				operation, sn95031_ctx->pll_state);
	}
}
EXPORT_SYMBOL_GPL(sn95031_configure_pll);

static int sn95031_set_vaud_bias(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: 0x%x\n", __func__, level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		pr_debug("vaud_bias PREPARE\n");
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			pr_debug("vaud_bias power up rail\n");
			/* power up the rail, on in normal and aoac mode */
			snd_soc_write(codec, SN95031_VAUD, 0x2D);
			usleep_range(1000, 1100);
		} else if (codec->dapm.bias_level == SND_SOC_BIAS_PREPARE) {
			pr_debug("vaud_bias STANDBY\n");
		}
		break;

	case SND_SOC_BIAS_OFF:
		pr_debug("vaud_bias OFF, doing rail shutdown\n");
		sn95031_configure_pll(codec, SN95031_DISABLE_PLL);
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
		usleep_range(1000, 1100);
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
		usleep_range(1000, 1100);
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
	SOC_DOUBLE_R("Headphone Playback Switch", SN95031_HSLVOLCTRL,
				SN95031_HSRVOLCTRL, 7, 1, 0),
	/* Add digital volume and mute controls for Speaker*/
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
	SOC_SINGLE("HSMIXER register", SN95031_HSMIXER, 0, 127, 0)
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

	/* Dummy widget to trigger VAUDA on/off */
	SND_SOC_DAPM_MICBIAS("VirtBias", SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route sn95031_audio_map[] = {
	/* headset and earpiece map */
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

	{ "VIB2OUT", NULL, "Vibra2 Playback"},
	{ "Vibra2 Playback", NULL, "Vibra2 Enable Mux"},
	{ "Vibra2 Enable Mux", "PWM", "Vibra2 DAC"},
	{ "Vibra2 Enable Mux", "SPI", "VIB2SPI"},

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
	{ "Mic_InputL Capture Route", "AMIC", "MIC1 Enable"},

	/* AMIC2 */
	{ "Mic_InputR Capture Route", "AMIC", "MIC2 Enable"},
#if (defined(CONFIG_SND_MFLD_MACHINE_GI) \
		|| defined(CONFIG_SND_MFLD_MACHINE_GI_MODULE))
	{ "MIC1 Enable", NULL, "AMIC2Bias"},
	{ "MIC2 Enable", NULL, "AMIC1Bias"},
	{ "AMIC1Bias", NULL, "AMIC2"},
	{ "AMIC2Bias", NULL, "AMIC1"},
#else
	{ "MIC1 Enable", NULL, "AMIC1Bias"},
	{ "MIC2 Enable", NULL, "AMIC2Bias"},
	{ "AMIC1Bias", NULL, "AMIC1"},
	{ "AMIC2Bias", NULL, "AMIC2"},
#endif
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

static int sn95031_codec_set_pll(struct snd_soc_codec *codec, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	int retval = 0;
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: 0x%x\n", __func__, source);
	mutex_lock(&codec->mutex);
	if (!freq_in || !freq_out) {
		/* disable PLL  */
		pr_debug("request to disable pll\n");
		sn95031_configure_pll(codec, SN95031_DISABLE_PLL);
		retval = 0;
		goto out;
	}
	if (sn95031_ctx->clk_src != source) {
		sn95031_ctx->pll_state = PLL_ENABLE_PENDING;
		sn95031_ctx->clk_src = source;
	}
	if (source == SN95031_INVALID)
		sn95031_ctx->pll_state = PLL_DISABLED;
out:
	mutex_unlock(&codec->mutex);
	return retval;
}

static int sn95031_set_pcm2_tristate(struct snd_soc_dai *codec_dai,
							int tristate)
{
	return snd_soc_update_bits(codec_dai->codec, SN95031_PCM2C2,
					BIT(0), !tristate);
}

static int sn95031_set_pcm1_tristate(struct snd_soc_dai *codec_dai,
							int tristate)
{
	return snd_soc_update_bits(codec_dai->codec, SN95031_PCM1C3,
						BIT(0), !tristate);
}


static int sn95031_codec_set_params(struct snd_soc_codec *codec,
						unsigned int param)
{
	unsigned int format;

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

static int sn95031_set_voice_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	int mode, format;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		mode = 0;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		mode = BIT(7);
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format = BIT(2);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		format = BIT(1)|BIT(0);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format = BIT(1);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		format = 0;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		format = BIT(0);
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, SN95031_PCM1C3, BIT(7), mode);
	snd_soc_update_bits(codec, SN95031_PCM1C2, BIT(0)|BIT(1)|BIT(2),
							format);
	return 0;
}

static int sn95031_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	unsigned int rate;

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
	pr_debug("%s: format=0x%x\n", __func__, params_format(params));
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
	pr_debug("%s: format=0x%x\n", __func__, format);
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
	/* enable PCM1 */
	sn95031_set_pcm1_tristate(dai, 0);
	return 0;
}

static int sn95031_voice_hw_free(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	pr_debug("%s\n", __func__);
	sn95031_set_pcm1_tristate(dai, 1);
	/* PCM1 should be in slave, short or long sync mode for
		Tx line to be in Hi-Z state */
	sn95031_set_voice_dai_fmt(dai, SND_SOC_DAIFMT_CBS_CFS);
	sn95031_set_voice_dai_fmt(dai, SND_SOC_DAIFMT_CBS_CFS
					| SND_SOC_DAIFMT_DSP_A);
	return 0;
}

/* Codec DAI section */
static const struct snd_soc_dai_ops sn95031_headset_dai_ops = {
	.digital_mute	= sn95031_pcm_hs_mute,
	.hw_params	= sn95031_pcm_hw_params,
	.set_tristate	= sn95031_set_pcm2_tristate,
};

static const struct snd_soc_dai_ops sn95031_speaker_dai_ops = {
	.digital_mute	= sn95031_pcm_spkr_mute,
	.hw_params	= sn95031_pcm_hw_params,
	.set_tristate	= sn95031_set_pcm2_tristate,
};

static const struct snd_soc_dai_ops sn95031_voice_dai_ops = {
	.digital_mute	= sn95031_pcm_hs_mute,
	.hw_params	= sn95031_voice_hw_params,
	.set_fmt	= sn95031_set_voice_dai_fmt,
	.hw_free	= sn95031_voice_hw_free,
	.set_tristate	= sn95031_set_pcm1_tristate,
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
	.ops = NULL,
},
{	.name = "SN95031 Vibra2",
	.playback = {
		.stream_name = "Vibra2",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SN95031_RATES,
		.formats = SN95031_FORMATS,
	},
	.ops = NULL,
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

/* codec registration */
static int sn95031_codec_probe(struct snd_soc_codec *codec)
{
	struct sn95031_priv *sn95031_ctx;

	pr_debug("codec_probe called\n");

	codec->dapm.idle_bias_off = 1;

	sn95031_ctx = kzalloc(sizeof(struct sn95031_priv), GFP_ATOMIC);
	if (!sn95031_ctx) {
		pr_err("codec ctx aloc failed\n");
		return -ENOMEM;
	}
	sn95031_ctx->clk_src = SN95031_INVALID;
	sn95031_ctx->pll_state = PLL_DISABLED;


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


	snd_soc_codec_set_drvdata(codec, sn95031_ctx);
	return 0;
}

static int sn95031_codec_remove(struct snd_soc_codec *codec)
{
	struct sn95031_priv *sn95031_ctx;
	sn95031_ctx = snd_soc_codec_get_drvdata(codec);

	pr_debug("codec_remove called\n");
	sn95031_set_vaud_bias(codec, SND_SOC_BIAS_OFF);

	kfree(sn95031_ctx);
	return 0;
}

struct snd_soc_codec_driver sn95031_codec = {
	.probe		= sn95031_codec_probe,
	.remove		= sn95031_codec_remove,
	.read		= sn95031_read,
	.write		= sn95031_write,
	.set_bias_level	= sn95031_set_vaud_bias,
	.idle_bias_off	= true,
	.set_params	= sn95031_codec_set_params,
	.dapm_widgets	= sn95031_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(sn95031_dapm_widgets),
	.dapm_routes	= sn95031_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(sn95031_audio_map),
	.set_pll	= sn95031_codec_set_pll,
	.controls	= sn95031_snd_controls,
	.num_controls	= ARRAY_SIZE(sn95031_snd_controls),
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

/*
module_platform_driver(sn95031_codec_driver);
*/
MODULE_DESCRIPTION("ASoC TI SN95031 codec driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sn95031");
